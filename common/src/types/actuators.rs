#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorsState {
    Armed(MotorOutputs),
    Disarmed(DisarmReason),
}

impl MotorsState {
    pub fn as_speeds(&self) -> [f32; 4] {
        match self {
            MotorsState::Armed(speeds) => speeds.0,
            _ => [0.0; 4],
        }
    }

    /// Maps the motor states to the classic PWM duty cycle representation.
    ///
    /// This is mostly useful for telemetry and logging.
    ///
    /// - Armed motors have values in the range `1000..=2000`
    /// - Disarmed motors have the value 0
    pub fn as_pwm_speeds(&self) -> [u16; 4] {
        match self {
            MotorsState::Armed(speeds) => speeds.0.map(|s| (s * 1000.0 + 1000.0) as u16),
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
pub struct MotorOutputs(pub [f32; 4]);

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DisarmReason {
    Uninitialized,
    ArmingBlocker,
    UserCommand,
    Killswitch,
    Timeout,
}
