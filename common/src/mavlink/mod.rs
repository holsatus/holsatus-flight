mod mission;

mod generator;
mod handler;
#[cfg(feature = "mavlink-osd")]
mod osd;
mod peers;

use core::{cell::RefCell, num::NonZeroU16, sync::atomic::Ordering};

use embassy_futures::{
    join,
    select::{select, Either},
};
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        NoopMutex,
    },
    channel::Channel,
    mutex::{MappedMutexGuard, Mutex, MutexGuard},
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use generator::Generator;
use mavio::{
    dialects::common::enums::MavModeFlag as MavModeInner,
    dialects::common::enums::MavState as MavStateInner,
    error::IoError,
    io::{AsyncReceiver, AsyncSender, EmbeddedIoAsyncReader, EmbeddedIoAsyncWriter},
    prelude::V2,
    protocol::FrameBuilder,
    Frame, Message,
};
use portable_atomic::AtomicU8;

use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::errors::{Debounce, MavlinkError};

// pub mod params;

pub struct MavMode {
    mode: AtomicU8,
}

impl MavMode {
    pub const fn new() -> Self {
        Self {
            mode: AtomicU8::new(0),
        }
    }

    pub fn modify(&self, func: impl Fn(&mut MavModeInner)) {
        _ = self
            .mode
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |raw_flag| {
                let mut mav_mode = MavModeInner::from_bits_truncate(raw_flag);

                func(&mut mav_mode);

                (raw_flag != mav_mode.bits()).then(|| {
                    try_make_request(Request::Single {
                        generator: Generator::Heartbeat,
                    });
                    mav_mode.bits()
                })
            });
    }

    pub fn get(&self) -> MavModeInner {
        let raw_flag = self.mode.load(Ordering::Relaxed);
        MavModeInner::from_bits_truncate(raw_flag)
    }
}

pub struct MavState {
    state: AtomicU8,
}

impl MavState {
    pub const fn new() -> Self {
        Self {
            state: AtomicU8::new(0),
        }
    }

    pub fn modify(&self, func: impl Fn(&mut MavStateInner)) {
        _ = self
            .state
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |raw_flag| {
                let mut mav_mode = MavStateInner::try_from(raw_flag).unwrap();

                func(&mut mav_mode);
                try_make_request(Request::Single {
                    generator: Generator::Heartbeat,
                });
                Some(mav_mode as u8)
            });
    }

    pub fn get(&self) -> MavStateInner {
        let raw_flag = self.state.load(Ordering::Relaxed);
        MavStateInner::try_from(raw_flag).unwrap()
    }
}

pub static MAV_STATE: MavState = MavState::new();

const NUM_STREAMERS: usize = 10;
const MAX_SERIAL: usize = 2;

pub async fn make_request(request: impl Into<Request>) {
    MAV_REQUEST.send(request.into()).await;
}

pub fn try_make_request(request: impl Into<Request>) {
    _ = MAV_REQUEST.try_send(request.into());
}

pub enum Request {
    /// Send a single message instance using the provided generator. Some messages
    /// may support sending all or a single instance, e.g. multiple sensors.
    Single {
        generator: Generator,
    },
    /// Start streaming messages of the type defined [`Generator`] at a specified
    /// interval defined by the `period_ms` parameter.
    StartStream {
        generator: Generator,
        period_ms: NonZeroU16,
    },
    /// Stop streaming the message specified by the [`Generator`] identifier.
    StopStream {
        generator: Generator,
    },
    SetSystemId {
        id: u8,
    },
    SetComponentId {
        id: u8,
    },
}

pub static MAV_MODE: MavMode = MavMode::new();

pub fn mav_set_armed(armed: bool) {
    MAV_MODE.modify(|mode| {
        mode.set(MavModeInner::SAFETY_ARMED, armed);
    });
}

pub fn get_mav_mode() -> MavModeInner {
    MAV_MODE.get()
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
    inner: &'static ServerInner,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct StreamCfg {
    pub period_ms: NonZeroU16,
    pub generator: Generator,
}

#[derive(Serialize, Deserialize)]
pub struct StreamCfgBundle {
    pub streams: [Option<StreamCfg>; NUM_STREAMERS],
}

impl Default for StreamCfgBundle {
    fn default() -> Self {
        let mut streams = [None; NUM_STREAMERS];

        streams[0] = Some(StreamCfg {
            period_ms: NonZeroU16::new(1000).unwrap(),
            generator: Generator::Heartbeat,
        });

        streams[1] = Some(StreamCfg {
            period_ms: NonZeroU16::new(100).unwrap(),
            generator: Generator::ServoOutputRaw,
        });

        streams[2] = Some(StreamCfg {
            period_ms: NonZeroU16::new(50).unwrap(),
            generator: Generator::AttitudeQuaternion,
        });

        streams[3] = Some(StreamCfg {
            period_ms: NonZeroU16::new(200).unwrap(),
            generator: Generator::GpsRawInt,
        });

        StreamCfgBundle { streams }
    }
}

struct MavReader<R: Read> {
    receiver: AsyncReceiver<IoError, EmbeddedIoAsyncReader<R>, V2>,
}

impl<R: Read> MavReader<R> {
    pub fn new(reader: R) -> Self {
        Self {
            receiver: AsyncReceiver::new::<V2>(EmbeddedIoAsyncReader::new(reader)),
        }
    }
}

struct MavSender<W: Write> {
    sender: AsyncSender<IoError, EmbeddedIoAsyncWriter<W>, V2>,
}

impl<W: Write> MavSender<W> {
    pub fn new(writer: W) -> Self {
        Self {
            sender: AsyncSender::new::<V2>(EmbeddedIoAsyncWriter::new(writer)),
        }
    }
}

pub static MAV_REQUEST: Channel<CriticalSectionRawMutex, Request, 2> = Channel::new();

pub type GenFn = fn(&MavServer) -> Result<Frame<V2>, MavlinkError>;

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
            }),
        }
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main(&self, reader: impl Read, writer: impl Write) -> ! {
        join::join4(
            self.main_reader(reader),
            self.main_writer(writer),
            self.request_handler(),
            self.microservice_heartbeat(),
        )
        .await
        .0
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
            let time = self
                .inner
                .peer_manager
                .lock(|locked| locked.borrow().find_first_timeout());

            match select(
                Timer::at(time.unwrap_or(Instant::MAX)),
                self.inner.peer_manager_refresh.wait(),
            )
            .await
            {
                Either::First(()) => {
                    self.inner.peer_manager.lock(|state| {
                        let mut peer_manager = state.borrow_mut();
                        for peer in peer_manager.iter_timeouts() {
                            warn!("Peer timed out: {}", peer.identity())
                        }

                        info!("Down to {} peers", peer_manager.num_peers());
                    });
                }
                Either::Second(()) => continue,
            }
        }
    }

    fn evaluate_with_peer_manager(&self, frame: &Frame<V2>) {
        self.inner.peer_manager.lock(|manager| {
            let mut manager = manager.borrow_mut();
            match manager.evaluate_mav_id(frame.into(), Instant::now()) {
                Ok(true) => {
                    self.inner.peer_manager_refresh.signal(());
                    info!(
                        "Added new peer: ({}, {})",
                        frame.system_id(),
                        frame.component_id()
                    )
                }
                Ok(false) => {
                    trace!("New message from peer")
                }
                Err(error) => {
                    self.inner.peer_manager_refresh.signal(());
                    crate::signals::register_error(error);
                    error!(
                        "Unable to add another peer: ({}, {})",
                        frame.system_id(),
                        frame.component_id()
                    )
                }
            }
        })
    }

    async fn reader_inner<R: Read>(&self, reader: &mut MavReader<R>) -> Result<(), MavlinkError> {
        // Read a raw MAVLink message from the serial port
        let frame = reader.receiver.recv().await?;
        self.evaluate_with_peer_manager(&frame);

        // Macro to implement match statement
        macro_rules! dispatch_handler {
            ($($type:ident),+ $(,)?) => {
                use $crate::mavlink::handler::message::MessageHandler;
                match frame.message_id() {
                    $(
                        ::mavio::dialects::common::messages::$type::ID =>
                        ::mavio::dialects::common::messages::$type::handler(self, frame).await?,
                    )+
                    id => warn!("No handler for message ID {}", id),
                }
            };
        }

        dispatch_handler!(
            Heartbeat,
            Ping,
            RcChannelsOverride,
            CommandInt,
            CommandLong,
            Tunnel,
            MissionItemInt,
        );

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
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };
        (0..NUM_STREAMERS)
            .into_iter()
            .for_each(|index| spawner.spawn(streamer_worker(*self, index).unwrap()));

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

    async fn request_handler_inner(
        &self,
        state: &mut [Option<StreamCfg>; NUM_STREAMERS],
    ) -> Result<(), MavlinkError> {
        match MAV_REQUEST.receive().await {
            Request::StartStream {
                period_ms,
                generator,
            } => {
                // First check if we are already streaming this ID, if not, find a free slot
                let Some(worker_idx) = state
                    .iter()
                    .position(|&x| x.is_some_and(|i| i.generator == generator))
                    .or_else(|| state.iter().position(|&x| x.is_none()))
                else {
                    error!("No more free streamer tasks (max {})", NUM_STREAMERS);
                    Err(MavlinkError::MaxStreamers)?
                };

                // Update state and start the streamer task
                state[worker_idx] = Some(StreamCfg {
                    period_ms,
                    generator,
                });
                self.inner.streams[worker_idx].signal(state[worker_idx]);
            }
            Request::StopStream { generator } => {
                let Some(worker_idx) = state
                    .iter()
                    .position(|&x| x.is_some_and(|i| i.generator == generator))
                else {
                    warn!(
                        "Streamer task for MAVLink index {:?} not running",
                        generator
                    );
                    return Ok(()); // Should this be an error?
                };

                state[worker_idx] = None;
                self.inner.streams[worker_idx].signal(None);

                return Ok(());
            }
            Request::Single { generator } => {
                self.write_gen_fn(generator.generator()).await?;
            }
            Request::SetSystemId { id } => {
                self.inner.system_id.store(id, Ordering::Relaxed);
            }
            Request::SetComponentId { id } => {
                self.inner.component_id.store(id, Ordering::Relaxed);
            }
        }

        Ok(())
    }

    async fn write_gen_fn(&self, gen_fn: GenFn) -> Result<(), MavlinkError> {
        self.write_message_inner(|| gen_fn(self)).await
    }

    async fn write_message(&self, message: &dyn Message) -> Result<(), MavlinkError> {
        self.write_message_inner(|| self.build_frame(message)).await
    }

    async fn write_message_inner(
        &self,
        make: impl Fn() -> Result<Frame<V2>, MavlinkError>,
    ) -> Result<(), MavlinkError> {
        // Lock the out-buffer
        let guard = self.inner.out_buffer.lock().await;

        let frame = make()?;
        let mapped = MutexGuard::map(guard, |buffer| buffer.insert(frame));

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
}

#[embassy_executor::task(pool_size = NUM_STREAMERS)]
pub async fn streamer_worker(server: MavServer, idx: usize) -> ! {
    let signal = &server.inner.streams[idx];

    'infinite: loop {
        // Receive initial message
        let Some(mut cfg) = signal.wait().await else {
            continue 'infinite;
        };

        debug!(
            "Starting streaming of {:?} at {} Hz",
            cfg.generator,
            1e3 / cfg.period_ms.get() as f32
        );

        // Create a ticker for keeping a stable frequency
        let mut ticker = Ticker::every(Duration::from_millis(cfg.period_ms.get() as u64));

        'streaming: loop {
            match select(signal.wait(), ticker.next()).await {
                // Configuration is None, go back and wait on updated configuration
                Either::First(None) => {
                    debug!("Stopping streaming of {:?}", cfg.generator);
                    break 'streaming;
                }

                // New configuration is Some, update the ticker and configuration
                Either::First(Some(new_cfg)) => {
                    cfg = new_cfg;
                    debug!(
                        "Setting streaming of {:?} at {} Hz",
                        cfg.generator,
                        1e3 / cfg.period_ms.get() as f32
                    );
                    ticker = Ticker::every(Duration::from_millis(cfg.period_ms.get() as u64));
                }

                // Ticker has elapsed, fetch and send the MAVLink message
                Either::Second(()) => {
                    // TODO - Handle errors in streamer tasks
                    _ = server.write_gen_fn(cfg.generator.generator()).await;
                }
            }
        }
    }
}
