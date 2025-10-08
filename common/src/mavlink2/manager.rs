use embassy_executor::Spawner;

use super::instance::{initialize_instance, InstanceHandle, NUM_INSTANCES};

struct Manager {
    instances: [InstanceHandle; NUM_INSTANCES],
}

impl Manager {
    pub fn new(spawner: Spawner) {
        let manager = Manager {
            instances: core::array::from_fn(|id| initialize_instance(spawner, id as u8)),
        };
    }
}

#[embassy_executor::task]
async fn manager_start(manager: Manager) -> ! {
    loop {}
}
