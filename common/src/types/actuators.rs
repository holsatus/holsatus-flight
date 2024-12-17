#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorsState {
    Arming,
    ArmedIdle,
    Armed([u16; 4]),
    Disarmed(DisarmReason),
}

impl MotorsState {
    pub fn as_speeds(&self) -> [u16; 4] {
        match self {
            MotorsState::Armed(speeds) => *speeds,
            _ => [0, 0, 0, 0],
        }
    }

    pub fn is_disarmed(&self) -> bool {
        matches!(self, MotorsState::Disarmed(_))
    }

    pub fn is_armed(&self) -> bool {
        matches!(self, MotorsState::Armed(_) | MotorsState::ArmedIdle)
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DisarmReason {
    Uninitialized,
    ArmingBlocker,
    UserCommand,
    Killswitch,
    Timeout,
}
