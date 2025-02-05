mod msg_generator;
mod command;
mod mission;
mod peers;

use core::{cell::RefCell, num::{NonZeroU16, NonZeroU32}, sync::atomic::Ordering};

use command::AnyCommand;
use embassy_futures::{join, select::{select, Either}};
use embassy_sync::{blocking_mutex::{raw::{CriticalSectionRawMutex, NoopRawMutex}, NoopMutex}, channel::Channel, mutex::{MappedMutexGuard, Mutex, MutexGuard}, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use mavio::{dialects::common::{enums::{MavCmd, MavModeFlag, MavResult}, messages::{AttitudeQuaternion, CommandAck, CommandInt, CommandLong, GpsRawInt, Heartbeat, MissionItemInt, Ping, RcChannelsOverride, ServoOutputRaw, Tunnel}}, error::IoError, io::{AsyncReceiver, AsyncSender, EmbeddedIoAsyncReader, EmbeddedIoAsyncWriter}, prelude::V2, protocol::FrameBuilder, Frame, Message};
use msg_generator::{MavSendable, Params};
use portable_atomic::{AtomicU8, AtomicUsize};


use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::errors::{Debounce, MavlinkError};

pub static MAV_MODE: AtomicU8 = AtomicU8::new(0);
pub static MAV_STATE: AtomicUsize = AtomicUsize::new(0);

const NUM_STREAMERS: usize = 10;

pub enum MavRequest {
    Stream { id: u32, period_us: u32 },
    Single { id: u32 },
    SetSysId { id: u8 },
    SetCompId { id: u8 },
}

pub fn mav_set_armed(armed: bool) {
    MAV_MODE.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |x| {
        if armed {
            Some(x | MavModeFlag::SAFETY_ARMED.bits())
        } else {
            Some(x & !MavModeFlag::SAFETY_ARMED.bits())
        }
    }).unwrap();

    // Ensure a fresh heartbeat is sent upon changing arming state
    _ = MAV_REQUEST.try_send(MavRequest::Single { id: Heartbeat::message_id() });
}

type M = NoopRawMutex;
struct ServerInner {

    /// The configuration of the streamer tasks.
    pub streams: [Signal<M, Option<StreamCfg>>; NUM_STREAMERS],

    /// The buffer used to serialize MAVLink messages into. The MutexGuard is
    /// passed to the `serialized signal` to notify the writer task that the
    /// buffer is ready to be written to the serial port. This also ensures that
    /// the serialized message cannot be overwritten before it is written out.
    pub out_buffer: Mutex<M, Option<Frame<V2>>>,
    pub serialized: Signal<M, MappedMutexGuard<'static, M, Frame<V2>>>,

    /// Peers known to this server. The instant is the peer closest to being
    /// timed out.
    pub peer_manager: NoopMutex<RefCell<peers::PeerManager>>,
    pub peer_manager_refresh: Signal<M, ()>,

    /// The configuration of the MAVLink server
    pub config: Mutex<M, NonZeroU16>,

    pub peer_timeout_ms: u32,

    /// The sequence number of the MAVLink messages
    pub sequence: AtomicU8,

    /// The system ID of the MAVLink messages sent from this server
    pub system_id: AtomicU8,

    /// The component ID of the MAVLink messages sent from this server
    pub component_id: AtomicU8,
}

#[derive(Clone, Copy)]
pub struct MavServer {
    inner: &'static ServerInner
}

#[derive(Clone, Copy)]
struct StreamCfg {
    params: Params,
    period_us: NonZeroU32,
    func: MavSendable,
}

#[derive(Serialize, Deserialize)]
pub struct MavStreamCfg {
    pub streams: [Option<(u32, u16)>; NUM_STREAMERS],
}

impl Default for MavStreamCfg {
    fn default() -> Self {
        let mut array = [None; NUM_STREAMERS];
        array[0] = Some((Heartbeat::message_id(), 1));
        array[1] = Some((ServoOutputRaw::message_id(), 10));
        array[2] = Some((AttitudeQuaternion::message_id(), 10));
        array[3] = Some((GpsRawInt::message_id(), 5));
        MavStreamCfg { streams: array }
    }
}

struct MavReader<R: Read> {
    receiver: AsyncReceiver<IoError, EmbeddedIoAsyncReader<R>, V2>,
}

impl <R: Read> MavReader<R> {
    pub fn new(reader: R) -> Self {
        Self {
            receiver: AsyncReceiver::new::<V2>(EmbeddedIoAsyncReader::new(reader)),
        }
    }
}

struct MavSender<W: Write> {
    sender: AsyncSender<IoError, EmbeddedIoAsyncWriter<W>, V2>,
}

impl <W: Write> MavSender<W> {
    pub fn new(writer: W) -> Self {
        Self {
            sender: AsyncSender::new::<V2>(EmbeddedIoAsyncWriter::new(writer)),
        }
    }
}

pub static MAV_REQUEST: Channel<CriticalSectionRawMutex, MavRequest, 2> =  Channel::new();

pub type GenFn = fn(&MavServer, &Params) -> Result<Frame<V2>, MavlinkError>;

#[cfg(feature = "mavlink")]
impl MavServer {
    pub fn new() -> Self {
        const SIGNAL: Signal<NoopRawMutex, Option<StreamCfg>> = Signal::new();
        static STATIC: StaticCell<ServerInner> = StaticCell::new();

        Self {
            inner: STATIC.init(ServerInner {
                streams: [SIGNAL; NUM_STREAMERS],
                out_buffer: Mutex::new(None),
                serialized: Signal::new(),
                peer_manager: NoopMutex::new(RefCell::new(peers::PeerManager::new())),
                peer_manager_refresh: Signal::new(),
                config: Mutex::new(NonZeroU16::new(1_000).unwrap()),
                peer_timeout_ms: 5_000,
                sequence: AtomicU8::new(0),
                system_id: AtomicU8::new(1),
                component_id: AtomicU8::new(1),
            })
        }
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main(&self, reader: impl Read, writer: impl Write) -> ! {
        join::join4(
            self.main_reader(reader),
            self.main_writer(writer),
            self.request_handler(),
            self.microservice_heartbeat(),
        ).await.0
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main_reader(&self, reader: impl Read) -> ! {
        info!("Starting MAVLink reader task");

        let mut debounce = Debounce::new(Duration::from_secs(1));
        let mut reader = MavReader::new(reader);

        loop {
            if let Err(error) = self.reader_inner(&mut reader).await {
                if let Some(error) = debounce.evaluate(error) {
                    error!("Mavlink reader error: {:?}", error);
                    crate::signals::register_error(error);
                }
            }
        }
    }

    async fn microservice_heartbeat(&self) -> ! {

        loop {
            let time = self.inner.peer_manager.lock(|locked|locked.borrow().find_first_timeout());

            if let Either::Second(()) = select(
                Timer::at(time.unwrap_or(Instant::MAX)),
                self.inner.peer_manager_refresh.wait()
            ).await {
                continue;
            }

            self.inner.peer_manager.lock(|state| {
                let mut peer_manager = state.borrow_mut();
                for peer in peer_manager.iter_timeouts() {
                    warn!("Peer timed out: {}", peer.mav_id())
                }

                info!("Down to {} peers", peer_manager.num_peers());
            });
        }
    }

    async fn reader_inner<R: Read>(&self, reader: &mut MavReader<R>) -> Result<(), MavlinkError> {

        // Read a raw MAVLink message from the serial port
        let frame = reader.receiver.recv().await?;

        // // Notify the peer manager that we have received a new message.
        // peers::PEER_MANAGER_MSG.send(PeerManagerMsg::HeartbeatFrom((&raw).into())).await;

        // match self.inner.peer_manager.lock().await.evaluate_mav_id(peers::MavId::from_raw_msg(&raw), Instant::now()) {
        //     Ok(true) => {
        //         self.inner.peer_manager_refresh.signal(());
        //         info!("Added new peer: ({}, {})",
        //             raw.system_id(), raw.component_id()
        //         )
        //     },
        //     Ok(false) => {
        //         trace!("New message from peer")
        //     },
        //     Err(error) => {
        //         self.inner.peer_manager_refresh.signal(());
        //         register_error(error);
        //         error!("Unable to add another peer: ({}, {})",
        //             raw.system_id(), raw.component_id()
        //         )
        //     },
        // }

        // Handle the message
        match frame.message_id() {

            id if id == Heartbeat::message_id() => {
                let _message = Heartbeat::try_from(frame.payload())?;
                // self.handle_rcv_heartbeat(raw.system_id(), raw.component_id(), message).await?;
            }

            id if id == Ping::message_id() => {
                let mut message = Ping::try_from(frame.payload())?;
                debug!("Received ping request from system {}", frame.system_id());

                // Update the ping message to the senders ID
                message.target_component = frame.component_id();
                message.target_system = frame.system_id();
                self.write_message(&message).await?;
            }

            id if id == RcChannelsOverride::message_id() => {
                let message = RcChannelsOverride::try_from(frame.payload())?;

                // Ensure the message is for this system
                if message.target_system != self.inner.system_id.load(Ordering::Relaxed) {
                    return Ok(());
                }

                let mut values = [990; 16];

                values[0] = message.chan1_raw;
                values[1] = message.chan2_raw;
                values[2] = message.chan3_raw;
                values[3] = message.chan4_raw;
                values[4] = message.chan5_raw;
                values[5] = message.chan6_raw;
                values[6] = message.chan7_raw;
                values[7] = message.chan8_raw;

                crate::signals::RC_CHANNELS_RAW.sender().send(Some(values));
            }

            id if id == CommandInt::message_id() || id == CommandLong::message_id() => {

                // NOTE If any message can only be handled by one of the two
                // command message definitions, it should be handled here with
                // an early return. Otherwise, the message should be handled the
                // same for both COMMAND_INT and COMMAND_LONG.

                let message: AnyCommand = if frame.message_id() == CommandInt::message_id() {
                    CommandInt::try_from(frame.payload())?.into()
                } else {
                    CommandLong::try_from(frame.payload())?.into()
                };

                let mav_cmd = message.command();
                debug!("Received command: {:?}", mav_cmd as u32);

                // NOTE Fulfilling these requests should probably be handled by
                // set set of "command workers" to avoid 'blocking' the main
                // MAVLink reader task for long-running commands. The workers
                // could be handled with the [´Procedure´] type.

                let result = match message.command() {
                    MavCmd::SetMessageInterval =>
                        command::set_message_interval(message).await,

                    MavCmd::DoSetActuator =>
                        command::do_set_actuator(message).await,

                    MavCmd::ComponentArmDisarm =>
                        command::component_arm_disarm(message).await,

                    MavCmd::RequestMessage =>
                        command::request_message(message).await,

                    _ => MavResult::Unsupported,    
                };

                let ack = CommandAck {
                    command: mav_cmd,
                    result,
                    progress: 100,
                    result_param2: 0,
                    target_system: frame.system_id(),
                    target_component: frame.component_id(),
                };

                self.write_message(&ack).await?;
            }

            id if id == Tunnel::message_id() => {
                let message = Tunnel::try_from(frame.payload())?;
                let slice = &message.payload[..message.payload_length as usize];
                debug!("Received tunneled data: {:?}", slice);
            }

            id if id == MissionItemInt::message_id() => {
                let message = MissionItemInt::try_from(frame.payload())?;
                mission::handle_mission_item_int(message).await?;
            }

            _ => warn!("No handler for message ID {}", frame.message_id()),
        }

        Ok(())
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main_writer(&self, writer: impl Write) -> ! {

        let mut writer = MavSender::new(writer);

        info!("Starting MAVLink writer task");

        let mut debounce = Debounce::new(Duration::from_secs(1));

        loop {
            let buffer = self.inner.serialized.wait().await;
            let result = writer.sender.send(&*buffer).await;


            if let Err(error) = result.map_err(MavlinkError::from) {
                if let Some(error) = debounce.evaluate(error) {
                    error!("Mavlink writer error: {:?}", error);
                    crate::signals::register_error(error);
                }
            }
        }
    }

    pub async fn request_handler(&self) -> ! {

        // Spawn all worker (streamers) tasks
        let spawner = embassy_executor::Spawner::for_current_executor().await;
        (0..NUM_STREAMERS).into_iter().for_each(|index| {
            spawner.must_spawn(streamer_worker(*self, index))
        });

        let mut state = [None; NUM_STREAMERS];
        let mut debounce = Debounce::new(Duration::from_secs(1));

        loop {
            if let Err(error) = self.request_handler_inner(&mut state).await {
                if let Some(error) = debounce.evaluate(error) {
                    error!("Mavlink writer error: {:?}", error);
                    crate::signals::register_error(error);
                }
            }
        }
    }

    async fn request_handler_inner(&self, state: &mut [Option<StreamCfg>; NUM_STREAMERS]) -> Result<(), MavlinkError> {
        match MAV_REQUEST.receive().await {
            MavRequest::Stream { id, period_us } => {

                let func = MavSendable::try_from(id)?;

                // If the frequency is zero, stop the streamer task
                let Ok(period_us) = NonZeroU32::try_from(period_us) else {

                    // Find the streamer task to stop
                    let Some(worker_idx) = state.iter().position(|&x| x.is_some_and(|i|i.func == func)) else {
                        warn!("Streamer task for MAVLink index {:?} not running", id);
                        return Ok(()) // Should this be an error?
                    };

                    state[worker_idx] = None;
                    self.inner.streams[worker_idx].signal(None);

                    return Ok(());
                };

                // First check if we are already streaming this ID, if not, find a free slot
                let Some(worker_idx) = state.iter().position(|&x| x.is_some_and(|i|i.func == func))
                .or_else(||state.iter().position(|&x| x.is_none())) else {
                    error!("No more free streamer tasks (max {})", NUM_STREAMERS);
                    Err(MavlinkError::MaxStreamers)?
                };

                // Update state and start the streamer task
                state[worker_idx] = Some(StreamCfg{period_us, func, params: Default::default()});
                self.inner.streams[worker_idx].signal(state[worker_idx]);
            },
            MavRequest::Single { id } => {
                let func = MavSendable::try_from(id)?;
                self.write_gen_fn(func.gen_fn(), &Default::default()).await?;
            },
            MavRequest::SetSysId { id } => {
                self.inner.system_id.store(id, Ordering::Relaxed);
            },
            MavRequest::SetCompId { id } => {
                self.inner.component_id.store(id, Ordering::Relaxed);
            },
        }

        Ok(())
    }

    async fn write_gen_fn(&self, gen_fn: GenFn, params: &Params) -> Result<(), MavlinkError> {
        let guard = self.inner.out_buffer.lock().await;

        let frame = gen_fn(self, params)?;
        let mapped = MutexGuard::map(guard, |buffer| {
            buffer.insert(frame)
        });

        self.inner.serialized.signal(mapped);
        Ok(())
    }

    async fn write_message(&self, message: &dyn Message) -> Result<(), MavlinkError> {

        let guard = self.inner.out_buffer.lock().await;

        let frame = self.build_frame(message)?;
        let mapped = MutexGuard::map(guard, |buffer| {
            buffer.insert(frame)
        });

        self.inner.serialized.signal(mapped);
        Ok(())
    }

    /// Construct a new header for outgoing MAVLink messages
    fn build_frame(&self, msg: &dyn Message) -> Result<mavio::Frame<V2>, MavlinkError> {
        let frame = FrameBuilder::new()
            .system_id(self.inner.system_id.load(Ordering::Relaxed))
            .component_id(self.inner.component_id.load(Ordering::Relaxed))
            .sequence(self.inner.sequence.fetch_add(1, Ordering::Relaxed))
            .version(V2)
            .message(msg)?
            .build();

        Ok(frame)
    }

    fn timeout_dur(&self) -> Duration {
        Duration::from_millis(self.inner.peer_timeout_ms as u64)
    }
}

#[cfg(feature = "mavlink")]
#[embassy_executor::task(pool_size = NUM_STREAMERS)]
pub async fn streamer_worker(server: MavServer, idx: usize) -> ! {

    let signal = &server.inner.streams[idx];

    'infinite: loop {

        // Receive initial message
        let Some(mut cfg) = signal.wait().await else {
            continue 'infinite;
        };

        debug!("Starting streaming of {:?} at {} Hz", cfg.func, 1e6 / cfg.period_us.get() as f32);

        // Create a ticker for keeping a stable frequency
        let mut ticker = Ticker::every(Duration::from_micros(cfg.period_us.get() as u64));

        'streaming: loop {

            match select(signal.wait(), ticker.next()).await {

                // Configuration is None, go back and wait on updated configuration
                Either::First(None) => {
                    debug!("Stopping streaming of {:?}", cfg.func);
                    break 'streaming
                }

                // New configuration is Some, update the ticker and configuration
                Either::First(Some(new_cfg)) => {
                    debug!("Setting streaming of {:?} at {} Hz", new_cfg.func, 1e6 / cfg.period_us.get() as f32);
                    ticker = Ticker::every(Duration::from_micros(new_cfg.period_us.get() as u64));
                    cfg = new_cfg;
                }

                // Ticker has elapsed, fetch and send the MAVLink message
                Either::Second(()) => {
                    // TODO - Handle errors in streamer tasks
                    _ = server.write_gen_fn(cfg.func.gen_fn(), &cfg.params).await;
                }
            }
        }
    }
}
