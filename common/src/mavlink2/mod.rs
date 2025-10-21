use core::num::Wrapping;

use embassy_executor::SendSpawner;
use maitake_sync::WaitMap;

use crate::errors::Debounce;
use crate::mavlink2::handler::Handler;
use crate::sync::channel::Channel;
use crate::{errors::adapter::embedded_io::EmbeddedIoError, mavlink2::messages::Generator};
use embassy_futures::select::{select, select3, select_array, Either, Either3};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use heapless::Vec;
use mavio::{
    prelude::{Versioned, V2},
    protocol::FrameParser,
    Frame,
};

use params::{Identity, Parameters};

use crate::serial::{IoStream, StreamId};

mod handler;
pub mod mav_mode;
pub mod mav_state;
mod messages;
mod microservice;
pub mod params;

/// Maximum number of ports/IO this MAVLink instance may use
const MAX_NUM_INTERFACES: usize = 2;

/// Maximum number of peers each port may remember.
const MAX_NUM_PEERS: usize = 4;

/// Maximum number of data streamers that can be active
const MAX_NUM_STREAMS: usize = 8;

macro_rules! enforce_mav_target {
    ($self:ident, $msg:ident) => {
        let target_id = Identity {
            sys: $msg.target_system,
            com: $msg.target_component,
        };

        if $self.param.id != target_id {
            return Ok(());
        }
    };
}

// The tasks state and saved parameters
struct MavlinkServer {
    param: Parameters,
    interfaces: MultiInterface,
    next_peer_timeout: Option<Instant>,
}

#[derive(Debug)]
struct Interface {
    stream: IoStream,
    stream_id: StreamId,
    parser: FrameParser<V2>,
    sequence: Wrapping<u8>,
    peers: Vec<Peer, MAX_NUM_PEERS>,
}

static STREAM_MAP: WaitMap<u8, Option<params::Stream>> = WaitMap::new();

#[embassy_executor::task(pool_size = MAX_NUM_STREAMS)]
async fn stream_runner(id: u8) {
    let mut opt_stream = None;
    loop {
        match select(STREAM_MAP.wait(id), async {
            match opt_stream {
                // Send the generator for broadcast and wait the duration.
                // Waiting the duration is not as precise timing-wise
                // as using a ticker, but it does degrade more gracefully
                // during high throughput.
                Some((generator, duration)) => loop {
                    CHANNEL
                        .send(Message::SendGenerator {
                            generator,
                            target: Target::Broadcast,
                        })
                        .await;
                    Timer::after(duration).await;
                },
                // We are not configured  to do anything, pend forever
                None => core::future::pending::<()>().await,
            }
        })
        .await
        {
            Either::First(Ok(new_stream)) => {
                // Map the message into something more useful
                opt_stream = new_stream.and_then(|stream| {
                    Some((
                        Generator::from_id(stream.message_id)?,
                        Duration::from_millis(stream.interval_ms as u64),
                    ))
                });
            }
            // The second future never returns by itself, the the WaitMap
            // is never closed, so we never get to this point in reality.
            _ => (),
        }
    }
}

struct MultiInterface([Option<Interface>; MAX_NUM_INTERFACES]);

impl MultiInterface {
    pub fn new() -> Self {
        Self([const { None }; MAX_NUM_INTERFACES])
    }

    pub fn _is_full(&self) -> bool {
        self.0.iter().all(|elem| elem.is_some())
    }

    pub fn push(&mut self, port: Interface) -> Result<(), Interface> {
        let mut index = 0;
        loop {
            match self.0.get_mut(index) {
                Some(port_opt) => {
                    if port_opt.is_none() {
                        *port_opt = Some(port);
                        return Ok(());
                    } else {
                        index += 1;
                        continue;
                    }
                }
                None => return Err(port),
            }
        }
    }

    /// Remove peers with the `peer_id` from all ports
    fn _remove_peer_all(&mut self, identity: Identity) {
        for port in self.iter_mut() {
            port.peers.retain(|peer| peer.identity != identity);
        }
    }

    /// Remove a peer from the specified port. This may be used when a peer
    /// has timed out or is no longer reachable on the specified port.
    fn _remove_peer(&mut self, stream_id: StreamId, peer_id: Identity) {
        if let Some(port) = self
            .iter_mut()
            .find(|port| port.stream.name().id() == stream_id)
        {
            port.peers.retain(|peer| peer.identity != peer_id);
        }
    }

    /// Find the next peer to timeout.
    ///
    /// This is used to determine when the server should check for
    /// inactive peers and remove them.
    fn find_next_peer_timeout(&mut self) -> Option<Instant> {
        let mut soonest_timeout = Instant::MAX;
        for port in self.iter() {
            if let Some(peer) = port.peers.iter().min_by_key(|p| p.last_seen) {
                if peer.last_seen < soonest_timeout {
                    soonest_timeout = peer.last_seen;
                }
            }
        }

        (soonest_timeout != Instant::MAX).then_some(soonest_timeout)
    }

    pub async fn recv_frame(&mut self) -> (Result<Frame<V2>, EmbeddedIoError>, StreamId) {
        let (result, index) = select_array(self.0.each_mut().map(|port_opt| async {
            match port_opt.as_mut() {
                Some(port) => loop {
                    let buf = port.parser.buffer_to_fill();
                    let bytes = port.stream.read(buf).await?;

                    if bytes == 0 {
                        return Err(EmbeddedIoError::UnexpectedEof);
                    }

                    if let Some(frame) = port.parser.commit_bytes(bytes) {
                        return Ok(frame);
                    }
                },
                None => core::future::pending().await,
            }
        }))
        .await;

        // This is not a very pretty way of getting the port_id
        (
            result,
            self.0
                .get(index)
                .unwrap()
                .as_ref()
                .unwrap()
                .stream
                .name()
                .id(),
        )
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Interface> {
        self.0.iter_mut().flatten()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Interface> {
        self.0.iter().flatten()
    }
}

impl Interface {
    fn new(stream: IoStream) -> Self {
        Self {
            stream_id: stream.name().id(),
            stream,
            parser: FrameParser::new(),
            sequence: Wrapping(0),
            peers: Vec::new(),
        }
    }

    pub fn has_peer(&self, peer_id: &Identity) -> bool {
        self.peers.iter().any(|p| p.identity == *peer_id)
    }

    pub async fn send_frame(
        &mut self,
        message: &dyn mavio::Message,
        param: &Parameters,
    ) -> Result<(), Error> {
        let frame = build_frame::<V2>(param, message, self.sequence.0)?;

        let mut buffer = [0u8; 280];
        let bytes = frame.serialize(&mut buffer)?;
        let slice = &buffer[..bytes];

        self.stream.write_all(slice).await.map_err(self.map_err())?;

        self.sequence += 1;

        Ok(())
    }

    /// Slightly crusty function to add an identifier to the io error.
    fn map_err(&self) -> impl Fn(EmbeddedIoError) -> Error + use<'_> {
        |err: EmbeddedIoError| Error::Io {
            err,
            stream_id: self.stream_id,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Peer {
    /// The Mavlink ID of the peer
    identity: Identity,
    /// The last time a message was seen from this peer
    last_seen: Instant,
    /// Number of messages received from this peer
    num_received: usize,
}

// Errors this task may produce
#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error("Received unsupported message ID: {id}")]
    UnsupportedMsg { id: u32 },
    #[error("Maximum number of peers ({max}) reached for {stream_id:?}")]
    MaxNumberPeers { max: u8, stream_id: StreamId },
    #[error("Maximum number of ports reached, max of {MAX_NUM_INTERFACES}")]
    MaxNumberPorts,
    #[error("Error on interface {err:?}: {stream_id:?}")]
    Io {
        err: EmbeddedIoError,
        stream_id: StreamId,
    },
    #[error("Reached EOF on interface {port_id:?}")]
    InterfaceEof { port_id: StreamId },
    #[error("Mavio spec error: {0:?}")]
    SpecError(mavio::error::SpecError),
    #[error("Mavio frame error: {0:?}")]
    FrameError(mavio::error::FrameError),
}

impl From<mavio::error::FrameError> for Error {
    fn from(error: mavio::error::FrameError) -> Self {
        Error::FrameError(error)
    }
}

impl From<mavio::error::SpecError> for Error {
    fn from(error: mavio::error::SpecError) -> Self {
        Error::SpecError(error)
    }
}

impl From<mavio::error::Error> for Error {
    fn from(error: mavio::error::Error) -> Self {
        match error {
            mavio::Error::Io(_) => unreachable!("We do not use mavio Io"),
            mavio::Error::Frame(frame_error) => Error::FrameError(frame_error),
            mavio::Error::Spec(spec_error) => Error::SpecError(spec_error),
        }
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Target {
    /// Send the message on all ports
    Broadcast,
    /// Send the message on all ports
    /// that are connected to the given peers
    /// (identified by their MavlinkId)
    Peers(Vec<Identity, MAX_NUM_PEERS>),
    /// Send the message on the given ports
    Ports(Vec<StreamId, MAX_NUM_INTERFACES>),
}

impl From<Identity> for Target {
    fn from(value: Identity) -> Self {
        Target::Peers(Vec::from_slice(&[value]).unwrap())
    }
}

impl From<StreamId> for Target {
    fn from(value: StreamId) -> Self {
        Target::Ports(Vec::from_slice(&[value]).unwrap())
    }
}

// Messages other tasks can send this task
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Message {
    SendGenerator {
        generator: messages::Generator,
        target: Target,
    },
    Foo,
}

static CHANNEL: Channel<Message, 5> = Channel::new();

pub async fn send(message: Message) {
    CHANNEL.send(message).await
}

#[embassy_executor::task]
pub async fn main(stream_id: &'static str) -> ! {
    MavlinkServer::new(stream_id).await.run().await
}

impl MavlinkServer {
    async fn new(stream_id: &'static str) -> Self {
        let spawner = SendSpawner::for_current_executor().await;

        // Spawn all streamer runners and yield once so they are guaranteed
        // to be ready ready once the parameters are loaded.
        for id in 0..MAX_NUM_STREAMS as u8 {
            spawner.spawn(stream_runner(id as u8).unwrap());
            embassy_futures::yield_now().await;
        }

        // The hearbeat microservice makes basic info available to others
        spawner.spawn(microservice::heartbeat::service_heartbeat().unwrap());

        // For debugging, spawn the task streaming certain messages
        spawner.spawn(messages::stream_position_set().unwrap());

        // Request and wait for the parameter table to be loaded
        let param = params::TABLE.read().await.clone();

        // Propagate all stream configurations to the streamer tasks
        for (id, stream) in param.stream.iter().enumerate() {
            STREAM_MAP.wake(&(id as u8), *stream);
        }

        // Claim a serial port to be used
        let io_stream = crate::serial::claim(stream_id).unwrap();
        let mut ports = MultiInterface::new();
        ports.push(Interface::new(io_stream)).unwrap();

        Self {
            param,
            interfaces: ports,
            next_peer_timeout: None,
        }
    }

    async fn run(&mut self) -> ! {
        let mut debounce = Debounce::new(Duration::from_secs(1));
        loop {
            if let Err(_error) = self.run_inner().await {
                if let Some(_) = debounce.evaluate(0u8) {
                    error!("[mavlink] Error logged");
                }
                // broadcast_error(error);
            }
        }
    }

    /// Returns a [`Timer`] future that expires at the soonest peer timeout.
    /// If there is no such peer timeout, this future will be pending forever.
    fn peer_timeout(&self) -> impl core::future::Future<Output = ()> {
        let next_peer_timeout = self.next_peer_timeout;
        async move {
            match next_peer_timeout {
                Some(timeout) => Timer::at(timeout).await,
                None => core::future::pending().await,
            }
        }
    }

    async fn run_inner(&mut self) -> Result<(), Error> {
        match select3(
            CHANNEL.receive(),
            self.peer_timeout(),
            self.interfaces.recv_frame(),
        )
        .await
        {
            // TODO We "block" receiving until a message is done sending. This can add major delays.
            Either3::First(message) => self.handle_message(message).await?,
            Either3::Second(()) => {
                warn!("[mavlink] A peer timed out, refreshing peer list");
                self.on_peer_timeout();
            }
            Either3::Third((Ok(frame), port_id)) => {
                self.handle_mav_frame(frame, port_id).await?;
            }
            Either3::Third((Err(error), port_id)) => {
                error!(
                    "[mavlink] Encountered an error reading from port: {:?}",
                    port_id
                );
                return Err(Error::Io {
                    err: error.into(),
                    stream_id: port_id,
                });
            }
        }

        Ok(())
    }

    async fn handle_message(&mut self, message: Message) -> Result<(), Error> {
        match message {
            Message::Foo => todo!(),
            Message::SendGenerator { generator, target } => {
                let message = generator.generate()?;
                self.send_mav_message(&message, target).await?;
            }
        }
        Ok(())
    }

    async fn send_mav_message(
        &mut self,
        message: &dyn mavio::Message,
        target: impl Into<Target>,
    ) -> Result<(), Error> {
        let send_target = target.into();
        match send_target {
            Target::Broadcast => {
                for port in self.interfaces.iter_mut() {
                    port.send_frame(message, &self.param).await?;
                }
            }
            Target::Peers(peer_ids) => {
                for peer_id in peer_ids.iter() {
                    for port in self.interfaces.iter_mut().filter(|l| l.has_peer(peer_id)) {
                        port.send_frame(message, &self.param).await?;
                    }
                }
            }
            Target::Ports(link_ids) => {
                for stream_id in link_ids.iter() {
                    if let Some(port) = self
                        .interfaces
                        .iter_mut()
                        .find(|l| l.stream.name().id() == *stream_id)
                    {
                        port.send_frame(message, &self.param).await?;
                    }
                }
            }
        }

        Ok(())
    }

    async fn handle_mav_frame(&mut self, frame: Frame<V2>, port_id: StreamId) -> Result<(), Error> {
        // Register the frame with the link ID, updating the peer list
        self.register_frame(&frame, port_id)?;

        use mavio::default_dialect::messages as m;
        match frame.message_id() {
            m::Heartbeat::ID => {
                let _msg = frame.decode_message::<m::Heartbeat>()?;
                let id = Identity::from(&frame);
                trace!("[mavlink] Got heartbeat from {:?}", id);
            }

            m::Ping::ID => {
                let mut msg = frame.decode_message::<m::Ping>()?;

                // If the target is non-zero, this is not a ping request
                if msg.target_system != 0 || msg.target_component != 0 {
                    return Ok(());
                }

                msg.target_system = frame.header().system_id();
                msg.target_component = frame.header().component_id();

                let target = Identity {
                    sys: frame.system_id(),
                    com: frame.component_id(),
                };

                self.send_mav_message(&msg, target).await?;
            }

            // Parameter-protocol related messages
            m::ParamRequestRead::ID => m::ParamRequestRead::handle(self, frame).await?,
            m::ParamRequestList::ID => m::ParamRequestList::handle(self, frame).await?,
            m::ParamSet::ID => m::ParamSet::handle(self, frame).await?,

            m::CommandInt::ID => {
                let msg = frame.decode_message::<m::CommandInt>()?;
                enforce_mav_target!(self, msg);
                error!("[mavlink] TODO: Support CommandInt messages");
                return Err(Error::UnsupportedMsg {
                    id: m::CommandInt::ID,
                });
            }
            m::CommandLong::ID => {
                let msg = frame.decode_message::<m::CommandLong>()?;
                enforce_mav_target!(self, msg);
                error!("[mavlink] TODO: Support CommandLong messages");
                return Err(Error::UnsupportedMsg {
                    id: m::CommandLong::ID,
                });
            }

            m::ViconPositionEstimate::ID => {
                let msg = frame.decode_message::<m::ViconPositionEstimate>()?;

                // The covariance matrices are bogus since they do not
                // seem to exist for the vicon tracker my Uni uses..
                let vicon_data = crate::types::measurements::ViconData {
                    timestamp_us: msg.usec,
                    position: [msg.x, msg.y, msg.z],
                    pos_var: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
                    attitude: [msg.roll, msg.pitch, msg.yaw],
                    att_var: [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]],
                };

                crate::signals::VICON_POSITION_ESTIMATE.send(vicon_data);
            }

            id => {
                warn!("[mavlink] Cannot handle unsupported message id: {}", id);
                return Err(Error::UnsupportedMsg { id });
            }
        }

        Ok(())
    }

    fn register_frame(&mut self, frame: &Frame<V2>, stream_id: StreamId) -> Result<(), Error> {
        let frame_seen = Instant::now();
        let frame_identity = Identity::from(frame);

        // TODO: Maybe spacial behavior for heartbeats?
        // let is_heartbeat = frame.message_id() == Heartbeat::ID;

        if let Some(interface) = self
            .interfaces
            .iter_mut()
            .find(|i| i.stream_id == stream_id)
        {
            match interface
                .peers
                .iter_mut()
                .find(|peer| peer.identity == frame_identity)
            {
                Some(existing_peer) => {
                    existing_peer.last_seen = frame_seen;
                    existing_peer.num_received += 1;
                }
                None => {
                    interface
                        .peers
                        .push(Peer {
                            identity: frame_identity,
                            last_seen: frame_seen,
                            num_received: 0,
                        })
                        .map_err(|_| {
                            error!(
                                "[mavlink] Too many peers registered for link {:?}",
                                stream_id
                            );
                            Error::MaxNumberPeers {
                                max: MAX_NUM_PEERS as u8,
                                stream_id,
                            }
                        })?;
                }
            }
        } else {
            error!("[mavlink] Invalid link ID: {:?}", stream_id);
        }

        if self.next_peer_timeout.is_none() {
            self.next_peer_timeout = Some(frame_seen + self.param.timeout());
        }

        Ok(())
    }

    fn on_peer_timeout(&mut self) {
        self.retain_active_peers();
        self.update_next_peer_timeout();
    }

    /// Remove all peers that have not been seen for a while.
    ///
    /// Peers will only be removed from the links they timed out on.
    fn retain_active_peers(&mut self) {
        let now = Instant::now();
        for interface in self.interfaces.iter_mut() {
            interface.peers.retain(|peer| {
                let retain = peer.last_seen + self.param.timeout() > now;
                if !retain {
                    debug!(
                        "[mavlink] Peer {:?} timed out on stream {:?}",
                        peer.identity, interface.stream_id
                    );
                }
                retain
            });
        }
    }

    /// Find the next peer to timeout.
    ///
    /// This is used to determine when the server should check for
    /// inactive peers and remove them.
    fn update_next_peer_timeout(&mut self) {
        self.next_peer_timeout = self
            .interfaces
            .find_next_peer_timeout()
            .map(|instant| instant + self.param.timeout())
    }
}

/// Build a MAVLink frame from the given parameters and message.
/// The sequence number is incremented after building the frame.
fn build_frame<V: Versioned>(
    param: &Parameters,
    message: &dyn mavio::Message,
    sequence: u8,
) -> Result<Frame<V>, Error> {
    let message = mavio::Frame::builder()
        .system_id(param.id.sys)
        .component_id(param.id.com)
        .sequence(sequence)
        .version(V::v())
        .message(message)
        .map_err(|e| match e {
            mavio::Error::Spec(spec_error) => spec_error,
            _ => unreachable!("This always produces a spec_error"),
        })?
        .build();

    Ok(message)
}
