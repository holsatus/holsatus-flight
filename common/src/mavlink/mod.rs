pub mod msg_generator;
pub mod command_int;

use core::{num::NonZeroU16, sync::atomic::Ordering};

use embassy_futures::{join, select::{select, Either}};
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, channel::Channel, mutex::{Mutex, MutexGuard}, signal::Signal};
use embassy_time::{Duration, Ticker};
use embedded_io::Error;
use embedded_io_async::{Read, Write};
use mavlink::{holsatus::MavMessage, read_v2_raw_message_async, MAVLinkV2MessageRaw, MessageData};
use msg_generator::MavSendable;
use portable_atomic::{AtomicU8, AtomicUsize};


use mavlink::holsatus::*;
use mavlink::MavlinkVersion;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::errors::{Debounce, MavlinkError};

pub static MAV_MODE: AtomicU8 = AtomicU8::new(0);
pub static MAV_STATE: AtomicUsize = AtomicUsize::new(0);

const NUM_STREAMERS: usize = 10;

pub enum MavRequest {
    Stream { id: u32, freq: u16 },
    Single { id: u32 },
    Stop { id: u32 },
}

pub fn mav_set_safety_armed(armed: bool) {
    MAV_MODE.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |x| {
        if armed {
            Some(x | MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED.bits())
        } else {
            Some(x & !MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED.bits())
        }
    }).unwrap();

    // Ensure a fresh heartbeat is sent upon changing arming state
    _ = MAV_REQUEST.try_send(MavRequest::Single { id: HEARTBEAT_DATA::ID });
}

type M = NoopRawMutex;
struct ServerInner {
    
    /// The configuration of the streamer tasks.
    pub streams: [Signal<M, Option<(StreamCfg, MavSendable)>>; NUM_STREAMERS],

    /// The buffer used to serialize MAVLink messages into. The MutexGuard is 
    /// passed to the `serialized signal` to notify the writer task that the
    /// buffer is ready to be written to the serial port. This also ensures that
    /// the serialized message cannot be overwritten before it is written out.
    pub out_buffer: Mutex<M, MAVLinkV2MessageRaw>,
    pub serialized: Signal<M, MutexGuard<'static, M, MAVLinkV2MessageRaw>>,

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
    freq: NonZeroU16,
    func: MavSendable,
}

#[derive(Serialize, Deserialize)]
pub struct MavStreamCfg {
    pub streams: [Option<(u32, u16)>; NUM_STREAMERS],
}

impl Default for MavStreamCfg {
    fn default() -> Self {
        let mut array = [None; NUM_STREAMERS];
        array[0] = Some((HEARTBEAT_DATA::ID, 1));
        array[1] = Some((SERVO_OUTPUT_RAW_DATA::ID, 10));
        array[2] = Some((ATTITUDE_QUATERNION_DATA::ID, 10));
        array[3] = Some((PID_INTERNALS_RATE_DATA::ID, 20));
        MavStreamCfg { streams: array }
    }
}

pub static MAV_REQUEST: Channel<CriticalSectionRawMutex, MavRequest, 2> =  Channel::new();

pub type GenFn = fn(&MavServer, &mut MAVLinkV2MessageRaw);

#[cfg(feature = "mavlink")]
impl MavServer {
    pub fn new() -> Self {
        const SIGNAL: Signal<NoopRawMutex, Option<(StreamCfg, MavSendable)>> = Signal::new();
        static STATIC: StaticCell<ServerInner> = StaticCell::new();

        Self { 
            inner: STATIC.init(ServerInner {
                streams: [SIGNAL; NUM_STREAMERS],
                out_buffer: Mutex::new(MAVLinkV2MessageRaw::new()),
                serialized: Signal::new(),
                sequence: AtomicU8::new(0),
                system_id: AtomicU8::new(1),
                component_id: AtomicU8::new(0),
            })
        }
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main(&self, reader: impl Read, writer: impl Write) -> ! {
        join::join(
            self.main_reader(reader),
            self.main_writer(writer),
        ).await.0
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main_reader(&self, mut reader: impl Read) -> ! {

        let mut debounce = Debounce::new(Duration::from_secs(1));

        loop {
            if let Err(error) = self.reader_inner(&mut reader).await {
                if let Some(error) = debounce.evaluate(error) {
                    error!("Mavlink reader error: {:?}", error);
                    crate::signals::register_error(error);
                }
            }
        }
    }

    async fn reader_inner(&self, mut reader: impl Read) -> Result<(), MavlinkError> {

        // Read a raw MAVLink message from the serial port
        let raw = read_v2_raw_message_async::<MavMessage>(&mut reader).await?;
        // Handle the message
        match raw.message_id() {
            HEARTBEAT_DATA::ID => {
                HEARTBEAT_DATA::deser(MavlinkVersion::V2, raw.payload())?;
                debug!("Received heartbeat from system {}",  raw.system_id());
            }

            PING_DATA::ID => {
                let mut message = PING_DATA::deser(MavlinkVersion::V2, raw.payload())?;
                debug!("Received ping request from system {}", raw.system_id());
                
                // Update the ping message to the senders ID
                message.target_component = raw.component_id();
                message.target_system = raw.system_id();
                self.write_msg_to_out_buffer(message).await;
            }

            RC_CHANNELS_OVERRIDE_DATA::ID => {
                let message = RC_CHANNELS_OVERRIDE_DATA::deser(MavlinkVersion::V2, raw.payload())?;

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

            COMMAND_INT_DATA::ID => {
                let message = COMMAND_INT_DATA::deser(MavlinkVersion::V2, raw.payload())?;

                match message.command {
                    MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL =>
                        command_int::set_message_interval(message).await?,
            
                    MavCmd::MAV_CMD_DO_SET_ACTUATOR => 
                        command_int::do_set_actuator(message).await?,
            
                    MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => 
                        command_int::component_arm_disarm(message).await?,
            
                    _ => Err(MavlinkError::UnknownMavCmd)?,
                }
            }

            _ => warn!("No handler for message ID {}", raw.message_id()),
        }

        Ok(())
    }

    /// This function is designed to be wrapped in an embassy task!
    pub async fn main_writer(&self, mut writer: impl Write) -> ! {

        // Spawn all worker (streamers) tasks
        let spawner = embassy_executor::Spawner::for_current_executor().await;
        (0..NUM_STREAMERS).into_iter().for_each(|index| {
            spawner.must_spawn(streamer_worker(*self, index))
        });
        
        let mut debounce = Debounce::new(Duration::from_secs(1));
        let mut state: [Option<StreamCfg>; 10] = [None; NUM_STREAMERS];

        loop {
            if let Err(error) = self.writer_inner(&mut writer, &mut state).await {
                if let Some(error) = debounce.evaluate(error) {
                    error!("Mavlink writer error: {:?}", error);
                    crate::signals::register_error(error);
                }
            }
        }
    }

    async fn writer_inner(&self, mut writer: impl Write, state: &mut [Option<StreamCfg>; 10]) -> Result<(), MavlinkError> {
        match select(
            MAV_REQUEST.receive(),
            self.inner.serialized.wait()
        ).await {
            Either::First(request) => match request {
                MavRequest::Stream { id, freq } => {

                    // If the frequency is zero, just stop the streamer task
                    let Ok(freq) = NonZeroU16::try_from(freq) else {
                        MAV_REQUEST.send(MavRequest::Stop { id }).await;
                        return Ok(());
                    };

                    let func = MavSendable::try_from(id)?;
    
                    // First check if we are already streaming this ID, if not, find a free slot
                    let Some(worker_idx) = state.iter().position(|&x| x.is_some_and(|i|i.func == func))
                    .or_else(||state.iter().position(|&x| x.is_none())) else {
                        error!("No more free streamer tasks (max {})", NUM_STREAMERS);
                        Err(MavlinkError::MaxStreamers)?
                    };
                    
                    // Update state and start the streamer task
                    state[worker_idx] = Some(StreamCfg{freq, func});
                    self.inner.streams[worker_idx].signal(state[worker_idx].map(|s|(s,func)));
                },
                MavRequest::Stop { id } => {
    
                    let func = MavSendable::try_from(id)?;
    
                    // Find the streamer task to stop
                    let Some(worker_idx) = state.iter().position(|&x| x.is_some_and(|i|i.func == func)) else {
                        warn!("Streamer task for MAVLink index {:?} not running", id);
                        return Ok(()) // Should this be an error?
                    };
    
                    state[worker_idx] = None;
                    self.inner.streams[worker_idx].signal(None);
                },
                MavRequest::Single { id } => {
                    let func = MavSendable::try_from(id)?;
                    self.write_to_buffer(func.gen_fn()).await
                },
            },
            Either::Second(to_write) => {
                writer.write_all(to_write.raw_bytes()).await
                    .map_err(|err|MavlinkError::IoError(err.kind().into()))?;
            },
        }

        Ok(())
    }

    async fn write_to_buffer<F>(&self, gen_function: F)
    where F: FnOnce(&MavServer, &mut MAVLinkV2MessageRaw)
    {
        let mut buffer = self.inner.out_buffer.lock().await;
        gen_function(self, &mut buffer);
        self.inner.serialized.signal(buffer);
    }

    async fn write_msg_to_out_buffer(&self, message: impl MessageData) {
        let mut buffer = self.inner.out_buffer.lock().await;
        buffer.serialize_message_data(self.new_header(), &message);
        self.inner.serialized.signal(buffer);
    }

    pub(crate) fn serialize(&self, raw: &mut MAVLinkV2MessageRaw, message: impl MessageData) {
        raw.serialize_message_data(self.new_header(), &message);
    }

    fn new_header(&self) -> mavlink::MavHeader {
        use core::sync::atomic::Ordering::Relaxed;
        mavlink::MavHeader {
            system_id: self.inner.system_id.load(Relaxed),
            component_id: self.inner.component_id.load(Relaxed),
            sequence: self.inner.sequence.fetch_add(1, Relaxed)
        }
    }
}

#[cfg(feature = "mavlink")]
#[embassy_executor::task(pool_size = NUM_STREAMERS)]
pub async fn streamer_worker(server: MavServer, idx: usize) -> ! {

    let signal = &server.inner.streams[idx];

    'infinite: loop {

        // Receive initial message
        let Some((mut cfg, mut func)) = signal.wait().await else {
            continue 'infinite;
        };
        debug!("Starting streaming of {:?} at {} Hz", cfg.func, cfg.freq);

        // Create a ticker for keeping a stable frequency
        let mut ticker = Ticker::every(Duration::from_hz(cfg.freq.get() as u64));

        'streaming: loop {

            match select(signal.wait(), ticker.next()).await {

                // Configuration is None, go back and wait on updated configuration
                Either::First(None) => {
                    debug!("Stopping streaming of {:?}", cfg.func);
                    break 'streaming 
                }

                // New configuration is Some, update the ticker and configuration
                Either::First(Some((new_cfg, new_func))) => {
                    debug!("Starting streaming of {:?} at {} Hz", new_cfg.func, new_cfg.freq);
                    ticker = Ticker::every(Duration::from_hz(new_cfg.freq.get() as u64));
                    cfg = new_cfg;
                    func = new_func;
                }

                // Ticker has elapsed, fetch and send the MAVLink message
                Either::Second(()) => {
                    server.write_to_buffer(func.gen_fn()).await;
                }
            }
        }
    }
}
