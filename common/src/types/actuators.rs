#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorsState {
    Armed(OutputsRaw),
    Disarmed(DisarmReason),
}

impl MotorsState {
    pub fn as_speeds(&self) -> [u16; 4] {
        match self {
            MotorsState::Armed(speeds) => speeds.0,
            _ => [0; 4],
        }
    }

    pub fn is_disarmed(&self) -> bool {
        matches!(self, MotorsState::Disarmed(_))
    }

    pub fn is_armed(&self) -> bool {
        matches!(self, MotorsState::Armed(_))
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputsRaw(pub [u16; 4]);

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DisarmReason {
    Uninitialized,
    ArmingBlocker,
    UserCommand,
    Killswitch,
    Timeout,
}
