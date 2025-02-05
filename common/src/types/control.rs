use serde::{Deserialize, Serialize};

/// Type which defines which RC control mode the drone should be in,
/// i.e. this is what determines what the sticks should make the drone do.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlMode {
    /// Rate mode, or acro mode, is most common for FPV pilots.
    /// This is more challenging for newcomers, but can provide
    /// a more fun and free flying experience.
    Rate,
    /// Angle mode, or horizon mode, will keep the drone level when
    /// there is no stick deflection, and map some stick position to
    /// some pitch/roll angle.
    Angle,
    /// Velocity mode is most common for cinematography, and will
    /// make the drone hold its position if no stick deflection.
    /// Some deflection will correspond to some velocity.
    ///
    /// **NOTE**: This is only available with a working GPS module.
    Velocity,

}

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDriver {
    Commander,
    RateLoop,
}

/// Analog RC control commands, e.g. a radio transmitter. In order,
/// the numbers represent, roll, pitch, yaw and throttle and 4 aux channels.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcAnalog(pub [f32; 8]);

impl RcAnalog {
    pub fn new(roll: f32, pitch: f32, yaw: f32, thrust: f32) -> Self {
        RcAnalog([roll, pitch, yaw, thrust, 0., 0., 0., 0.])
    }

    pub fn roll_pitch_yaw(&self) -> [f32; 3] {
        [self.0[0], self.0[1], self.0[2]]
    }

    pub fn throttle(&self) -> f32 {
        self.0[3]
    }

    pub fn aux(&self) -> [f32; 4] {
        [self.0[4], self.0[5], self.0[6], self.0[7]]
    }
}
