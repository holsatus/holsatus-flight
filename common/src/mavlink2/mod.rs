use embassy_time::Instant;
use heapless::Vec;
use portable_atomic::AtomicU8;

pub mod instance;
pub mod manager;
pub mod peer;
pub mod serial_dummy;

use crate::sync::channel::Channel;

/// Represents a single MAVLink peer
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Peer {
    mav_id: Identity,
    last_seen: Instant,
}

static SYSTEM_ID: AtomicU8 = AtomicU8::new(1);
static COMPONENT_ID: AtomicU8 = AtomicU8::new(1);

/// The number of Mavlink instances. This number of instances is
/// statically allocated, so make the number as small as necessary.
const MAX_NUM_PORTS: usize = 2;

/// The maximum number of peers that can be connected at once.
const MAX_NUM_PEERS: usize = 4;

static MANAGER_CHANNEL: Channel<ManagerMsg, 2> = Channel::new();

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Identity {
    system: u8,
    component: u8,
}

enum ManagerMsg {}

struct InstanceManager {
    instance: Vec<Instance, MAX_NUM_PORTS>,
    peers: Vec<Peer, MAX_NUM_PEERS>,
}

impl InstanceManager {
    pub fn new() -> Self {
        InstanceManager {
            instance: Vec::new(),
            peers: Vec::new(),
        }
    }
}

#[embassy_executor::task]
async fn run_instance_manager() -> ! {
    let mut manager = InstanceManager::new();

    loop {}
}

enum InstanceMsg {}

struct Instance {
    channel: &'static Channel<InstanceMsg, 2>,
    instance_id: u8,
}

struct InstanceInfo {
    channel: &'static Channel<InstanceMsg, 2>,
    instance_id: u8,
}

struct InstanceState {}

#[embassy_executor::task(pool_size = MAX_NUM_PORTS)]
async fn run_single_instance() -> ! {
    loop {}
}
