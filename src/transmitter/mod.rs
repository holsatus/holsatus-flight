use core::ops::{Deref, DerefMut};

pub mod tx_12_profiles;

#[derive(Debug, Clone, Copy, Default)]
pub struct ControlRequest {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub thrust: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct TransmitterMap ([ChannelType; 16]);

impl Deref for TransmitterMap {
    type Target = [ChannelType; 16];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for TransmitterMap {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

// TODO Move to a more appropriate location
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EventRequest {

    /// Unbound request // TODO remove?
    Unbound,

    /// Arm all motors
    ArmMotors,

    /// Disarm all motors. Acts as a kill-switch
    DisarmMotors,

    /// Change stabilization mode to angle (or horizon) mode
    AngleMode,

    /// Change stabilization mode to rate (or acro / 3D) mode
    RateMode,

    /// Start gyroscope calibration
    StartGyrCalib,

    /// Abort any currently active gyroscope calibration
    AbortGyrCalib,

    /// Start accelerometer calibration
    StartAccCalib,

    /// Abort any currently active accelerometer calibration
    AbortAccCalib,

    /// Start magnetometer calibration
    StartMagCalib,

    /// Abort any currently active magnetometer calibration
    AbortMagCalib,

    /// Save any currently modified parameters to flash
    SaveConfig,

    /// RC Failsafe
    RcFailsafe,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AnalogCommand {
    Roll,
    Pitch,
    Yaw,
    Thrust,
}

#[derive(Clone, Copy, Debug)]
pub enum CommandType {
    AnalogCommand,
    DiscreteCommand,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ChannelType {
    None,
    Analog((AnalogCommand, AnalogConfig)),
    Discrete([(u16, EventRequest); 3]),
}

// TODO Support expo curves and other non-linear mappings
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnalogConfig {
    in_min: u16,
    in_max: u16,
    deadband: u16,
    fullrange: bool,
    reverse: bool,
}

/// Maps the raw input value from RC according to the provided configuration.
pub fn analog_map(data: u16, cfg: &AnalogConfig) -> f32 {
    
    // Select range mapping function
    let value = if cfg.fullrange {
        analog_map_full(data, cfg)
    } else {
        analog_map_half(data, cfg)
    };

    // Reverse the value if needed
    if cfg.reverse {
        -value
    } else {
        value
    }
}

/// Full-range, maps the input range to the output range [-1, 1].
/// Typically used for roll, pitch and yaw commands.
fn analog_map_full(data: u16, cfg: &AnalogConfig) -> f32 {
    
    // Center the value around 0
    let mut value = data.saturating_sub(cfg.in_min) as i32 - (cfg.in_max - cfg.in_min) as i32/2;

    // Apply deadband on small values
    if value.abs() < cfg.deadband as i32 { value = 0 }

    // Normalize and clamp the value to the range [-1, 1]
    ((2*value) as f32 / (cfg.in_max - cfg.in_min) as f32).clamp(-1., 1.)
}

/// Half-range, maps the input range to the output range [0, 1].
/// Typically used for thrust commands.
pub fn analog_map_half(data: u16, cfg: &AnalogConfig) -> f32 {
    
    // Shift the value down to 0
    let mut value = data.saturating_sub(cfg.in_min);

    // Apply deadband on small values
    if value < cfg.deadband { value = 0 }

    // Normalize and clamp the value to the range [0, 1]
    (value as f32 / (cfg.in_max - cfg.in_min) as f32).clamp(0., 1.)
}