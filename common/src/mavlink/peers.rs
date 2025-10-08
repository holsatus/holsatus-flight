use embassy_futures::select::{select3, Either3};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use mavio::{prelude::Versioned, Frame};

use crate::errors::MavlinkError;

use super::generator::Generator;

pub const MAX_MAV_PEERS: usize = 4;
pub const TIMEOUT_DUR: Duration = Duration::from_secs(4);

pub enum PeerManagerMsg {
    HeartbeatFrom(Identity),
    SetHeartbeatRate(u8),
}

pub static PEER_MANAGER_MSG: Channel<ThreadModeRawMutex, PeerManagerMsg, 2> = Channel::new();

#[embassy_executor::task]
async fn peer_manager() -> ! {
    let peer_manager = PeerManager::new();
    let mut heartbeat_dur = Duration::from_secs(1);
    let mut heartbeat_tick = Instant::now();

    loop {
        match select3(
            PEER_MANAGER_MSG.receive(),
            peer_manager.await_timeout(),
            Timer::at(heartbeat_tick),
        )
        .await
        {
            Either3::First(msg) => match msg {
                PeerManagerMsg::HeartbeatFrom(_mav_id) => {}
                PeerManagerMsg::SetHeartbeatRate(rate) => {
                    if rate == 0 {
                        heartbeat_tick = Instant::MAX;
                    } else {
                        heartbeat_dur = Duration::from_hz(rate as u64);
                        heartbeat_tick += heartbeat_dur;
                    }
                }
            },
            Either3::Second(()) => {}
            Either3::Third(()) => {
                heartbeat_tick += heartbeat_dur;
                super::MAV_REQUEST
                    .send(super::Request::Single {
                        generator: Generator::Heartbeat,
                    })
                    .await;
            }
        }
    }
}

/// Describes the MAVLink system and component IDs
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Identity {
    pub system_id: u8,
    pub component_id: u8,
}

impl<V: Versioned> Into<Identity> for Frame<V> {
    fn into(self) -> Identity {
        (&self).into()
    }
}

impl<V: Versioned> Into<Identity> for &Frame<V> {
    fn into(self) -> Identity {
        Identity {
            system_id: self.system_id(),
            component_id: self.component_id(),
        }
    }
}

/// Represents a single MAVLink peer
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Peer {
    identity: Identity,
    last_seen: Instant,
}

impl Peer {
    pub fn identity(&self) -> Identity {
        self.identity
    }
}

/// Manages multiple peers, ensuring they are registered
/// and unregistered appropriately.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeerManager {
    peers: heapless::Vec<Peer, MAX_MAV_PEERS>,
    first_timeout: Option<Instant>,
}

impl PeerManager {
    pub fn new() -> Self {
        PeerManager {
            peers: heapless::Vec::new(),
            first_timeout: None,
        }
    }

    /// Obtain a mutable reference to the [`Peer`] of the given [`MavId`]
    pub fn peer_from_id_mut(&mut self, mav_id: Identity) -> Option<&mut Peer> {
        self.peers.iter_mut().find(|p| p.identity == mav_id)
    }

    /// Evaluate the MAVLink ID of a received message.
    ///
    /// If the ID is of a known [`Peer`], we simply update the time of the last received
    /// message and return `Ok(false)`. If the ID does not belong to a known peer, we
    /// instead add it to the list, and this function returns `Ok(true)`.
    pub fn evaluate_mav_id(
        &mut self,
        mav_id: Identity,
        time: Instant,
    ) -> Result<bool, MavlinkError> {
        if let Some(peer) = self.peer_from_id_mut(mav_id) {
            peer.last_seen = time;
            Ok(false)
        } else {
            self.peers
                .push(Peer {
                    identity: mav_id,
                    last_seen: time,
                })
                .map_err(|_| MavlinkError::MaxPeers)?;
            Ok(true)
        }
    }

    /// Finds soonest time where a peer will time out, if no new messages are received.
    pub fn find_first_timeout(&self) -> Option<Instant> {
        let earliest = self
            .peers
            .iter()
            .fold(None, |k: Option<Instant>, p: &Peer| {
                if k.is_none_or(|k| p.last_seen < k) {
                    Some(p.last_seen)
                } else {
                    k
                }
            });

        earliest.map(|t| t + TIMEOUT_DUR)
    }

    pub fn num_peers(&self) -> usize {
        self.peers.len()
    }

    /// Future which returns once a peer times out.
    ///
    /// If there are no peers, this will never return.
    /// This is intended to be used in a `select` expression, where at least one of the other arms
    /// wait for incoming messages, including heartbeats. If a heartbeat is returned, use the
    /// [`evaluate_id`] method to add a new peer, or update information about an existing peer.
    /// If the function does return, one or more peers has timed out, and you may use the
    /// [`iter_timedout`] function to iterate through and remove all timed out peers.
    pub async fn await_timeout(&self) {
        let maybe_timeout = self.find_first_timeout().map(|t| t + TIMEOUT_DUR);
        let timeout = maybe_timeout.unwrap_or(Instant::MAX);
        Timer::at(timeout).await;
    }

    pub fn iter_timeouts(&mut self) -> IterTimeouts<'_> {
        IterTimeouts {
            inner: self,
            checked: 0,
        }
    }
}

pub struct IterTimeouts<'a> {
    inner: &'a mut PeerManager,
    checked: usize,
}

impl<'a> Iterator for IterTimeouts<'a> {
    type Item = Peer;

    fn next(&mut self) -> Option<Self::Item> {
        let time_now = Instant::now();

        for (i, peer) in self.inner.peers.iter().skip(self.checked).enumerate() {
            self.checked = i;
            if peer.last_seen + TIMEOUT_DUR < time_now {
                return Some(self.inner.peers.swap_remove(i));
            }
        }

        None
    }
}

#[cfg(test)]
pub mod test {
    use embassy_time::Instant;

    use super::{Identity, Peer, PeerManager};

    #[test]
    fn test_iter_timeout() {
        let mut manager = PeerManager::new();

        let peer = Peer {
            identity: Identity {
                system_id: 255,
                component_id: 1,
            },
            last_seen: Instant::from_secs(0),
        };

        assert_eq!(manager.peer_from_id_mut(peer.identity), None);
    }
}
