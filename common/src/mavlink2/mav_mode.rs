///! Module for interacting with the systems MAV_MODE bitflag.
use core::sync::atomic::{AtomicU8, Ordering};
use mavio::default_dialect::enums::MavModeFlag;

pub struct MavMode {
    mode: AtomicU8,
}

impl MavMode {
    pub const fn new() -> Self {
        Self {
            mode: AtomicU8::new(0),
        }
    }

    pub fn modify(&self, func: impl Fn(&mut MavModeFlag)) {
        _ = self
            .mode
            .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |raw_flag| {
                let mut mav_mode = MavModeFlag::from_bits_truncate(raw_flag);

                func(&mut mav_mode);

                (raw_flag != mav_mode.bits()).then(|| {
                    if super::CHANNEL
                        .try_send(super::Message::SendGenerator {
                            generator: super::messages::Generator::Heartbeat,
                            target: super::Target::Broadcast,
                        })
                        .is_err()
                    {
                        warn!("Failed to send automatic heartbeat, skipping")
                    };
                    mav_mode.bits()
                })
            });
    }

    pub fn get(&self) -> MavModeFlag {
        let raw_flag = self.mode.load(Ordering::Relaxed);
        MavModeFlag::from_bits_truncate(raw_flag)
    }
}

static MAV_MODE: MavMode = MavMode::new();

pub fn set(flag: MavModeFlag, value: bool) {
    MAV_MODE.modify(|mode| mode.set(flag, value));
}

pub fn get() -> MavModeFlag {
    MAV_MODE.get()
}

pub fn contains(flag: MavModeFlag) -> bool {
    MAV_MODE.get().contains(flag)
}
