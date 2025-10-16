///! Module for interacting with the systems MAV_STATE bitflag.
use core::sync::atomic::{AtomicU8, Ordering};
use mavio::dialects::common::enums::MavState as MavStateInner;

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
                let mut mav_state = MavStateInner::try_from(raw_flag).unwrap();

                func(&mut mav_state);

                (raw_flag != mav_state as u8).then(|| {
                    if super::CHANNEL
                        .try_send(super::Message::SendGenerator {
                            generator: super::messages::Generator::Heartbeat,
                            target: super::Target::Broadcast,
                        })
                        .is_err()
                    {
                        warn!("Failed to send automatic heartbeat, skipping")
                    };
                    mav_state as u8
                })
            });
    }

    pub fn get(&self) -> MavStateInner {
        let raw_flag = self.state.load(Ordering::Relaxed);
        MavStateInner::try_from(raw_flag).unwrap()
    }
}

pub static MAV_STATE: MavState = MavState::new();

pub fn get() -> MavStateInner {
    MAV_STATE.get()
}

pub fn set(new_state: MavStateInner) {
    MAV_STATE.modify(|state| *state = new_state);
}
