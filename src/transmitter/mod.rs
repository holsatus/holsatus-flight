use core::ops::{Deref, DerefMut};

use num_traits::Float;

use crate::t_commander::CommanderRequest;

pub mod tx_12_profiles;

#[derive(Debug, Clone, Copy, Default)]
pub struct ControlRequest {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub thrust: f32,
}

impl ufmt::uDisplay for TransmitterMap {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where W: ufmt::uWrite + ?Sized
    {
        for (i, ch) in self.0.iter().enumerate() {
            match ch {
                ChannelType::None => {
                    ufmt::uwriteln!(f, "\nchannel_{}: null",i)?;
                },
                ChannelType::Analog(analog) => {
                    ufmt::uwriteln!(f, "\nchannel_{}:",i)?;
                    let (cmd, cfg) = analog;
                    let cmd_str = match cmd {
                        AnalogCommand::Roll => "Roll",
                        AnalogCommand::Pitch => "Pitch",
                        AnalogCommand::Yaw => "Yaw",
                        AnalogCommand::Thrust => "Thrust",
                    };

                    ufmt::uwriteln!(f, "  analog:")?;
                    ufmt::uwriteln!(f, "    signal: \"{}\"",cmd_str)?;
                    ufmt::uwriteln!(f, "    config:")?;
                    ufmt::uwriteln!(f, "      in_min: {}",cfg.in_min)?;
                    ufmt::uwriteln!(f, "      in_max: {}",cfg.in_max)?;
                    ufmt::uwriteln!(f, "      fullrange: {}",cfg.fullrange)?;
                    ufmt::uwriteln!(f, "      reverse: {}",cfg.reverse)?;
                    ufmt::uwriteln!(f, "      deadband: {}",cfg.deadband)?;
                    match cfg.rates {
                        Rates::None => {
                            ufmt::uwriteln!(f, "      rates: null")?;
                        },
                        Rates::Standard(rates) => {
                            let rate_int = (rates.rate*10000.) as i32;
                            let expo_int = (rates.expo*10000.) as i32;
                            let slow_int = (rates.slow*10000.) as i32;

                            let rate_whole = rate_int / 10000;
                            let rate_frac = rate_int - rate_whole * 10000;
                            let expo_whole = expo_int / 10000;
                            let expo_frac = expo_int - expo_whole * 10000;
                            let slow_whole = slow_int / 10000;
                            let slow_frac = slow_int - slow_whole * 10000;

                            ufmt::uwriteln!(f, "      rates:")?;
                            ufmt::uwriteln!(f, "        rate: {}.{}",rate_whole,rate_frac)?;
                            ufmt::uwriteln!(f, "        expo: {}.{}",expo_whole,expo_frac)?;
                            ufmt::uwriteln!(f, "        slow: {}.{}",slow_whole,slow_frac)?;
                        },
                    }
                },
                ChannelType::Discrete(discrete) => {
                    ufmt::uwriteln!(f, "\nchannel_{}:",i)?;
                    ufmt::uwriteln!(f, "  discrete:")?;
                    for (i,(val, req)) in discrete.iter().enumerate() {
                        ufmt::uwriteln!(f, "    binding_{}:",i)?;
                        let req_str = match req {
                            EventRequest::Unbound => "Unbound",
                            EventRequest::ArmMotors => "ArmMotors",
                            EventRequest::DisarmMotors => "DisarmMotors",
                            EventRequest::AngleMode => "AngleMode",
                            EventRequest::RateMode => "RateMode",
                            EventRequest::StartGyrCalib => "StartGyrCalib",
                            EventRequest::AbortGyrCalib => "AbortGyrCalib",
                            EventRequest::StartAccCalib => "StartAccCalib",
                            EventRequest::AbortAccCalib => "AbortAccCalib",
                            EventRequest::StartMagCalib => "StartMagCalib",
                            EventRequest::AbortMagCalib => "AbortMagCalib",
                            EventRequest::SaveConfig => "SaveConfig",
                            EventRequest::RcFailsafe => "RcFailsafe",
                        };
                        ufmt::uwriteln!(f, "      val: {}", val)?;
                        ufmt::uwriteln!(f, "      command: \"{}\"", req_str)?;


                        /*
                        ufmt::uwriteln!(f, "\nchannel_{}:",i)?;
                        ufmt::uwriteln!(f, "  discrete:")?;
                        for (val, req) in discrete.iter() {
                        let req_str = match req {
                            EventRequest::Unbound => "Unbound",
                            EventRequest::ArmMotors => "ArmMotors",
                            EventRequest::DisarmMotors => "DisarmMotors",
                            EventRequest::AngleMode => "AngleMode",
                            EventRequest::RateMode => "RateMode",
                            EventRequest::StartGyrCalib => "StartGyrCalib",
                            EventRequest::AbortGyrCalib => "AbortGyrCalib",
                            EventRequest::StartAccCalib => "StartAccCalib",
                            EventRequest::AbortAccCalib => "AbortAccCalib",
                            EventRequest::StartMagCalib => "StartMagCalib",
                            EventRequest::AbortMagCalib => "AbortMagCalib",
                            EventRequest::SaveConfig => "SaveConfig",
                            EventRequest::RcFailsafe => "RcFailsafe",
                        };
                        ufmt::uwriteln!(f, "    - [{}, \"{}\"]",val, req_str)?;
                        */
                    }
                },
            };
        }

        Ok(())
    }
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

impl TransmitterMap {
    pub fn sanity_check(&self) -> Result<(), (usize, RatesError)> {
        for (ch, channel) in self.iter().enumerate() {
            match channel {
                ChannelType::Analog((_, cfg)) =>  {
                    if let Err(e) = cfg.sanity_check_rates() {
                        return Err((ch, e));
                    }
                }
                _ => {}, // Currently no other checks for other channel types
            }
        }
        Ok(())
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

impl From<EventRequest> for CommanderRequest {
    fn from(value: EventRequest) -> Self {

        match value {
            EventRequest::Unbound => unreachable!("Unbound events should never reach the commander"),
            EventRequest::ArmMotors => CommanderRequest::ArmMotors(true),
            EventRequest::DisarmMotors => CommanderRequest::ArmMotors(false),
            EventRequest::AngleMode => CommanderRequest::AngleMode,
            EventRequest::RateMode => CommanderRequest::RateMode,
            EventRequest::StartGyrCalib => CommanderRequest::CalGyrCommand(crate::t_gyr_calibration::GyrCalCommand::Start(
                // Use all defaule values
                crate::t_gyr_calibration::CalGyrConfig {
                    sensor: None,
                    duration: None,
                    max_var: None,
                }
            )),
            EventRequest::AbortGyrCalib => CommanderRequest::CalGyrCommand(crate::t_gyr_calibration::GyrCalCommand::Stop),
            EventRequest::StartAccCalib => CommanderRequest::StartAccCalib {
                sensor: None,
                duration: None,
                max_var: None,
            },
            EventRequest::AbortAccCalib => CommanderRequest::AbortAccCalib,
            EventRequest::StartMagCalib => CommanderRequest::StartMagCalib {
                sensor: None,
                max_var: None,
            },
            EventRequest::AbortMagCalib => CommanderRequest::AbortMagCalib,
            EventRequest::SaveConfig => CommanderRequest::SaveConfig {
                config_id: None,
            },
            EventRequest::RcFailsafe => CommanderRequest::RcFailsafe,
        }
    }
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AnalogConfig {
    in_min: u16,
    in_max: u16,
    deadband: u16,
    fullrange: bool,
    reverse: bool,
    rates: Rates,
}

impl AnalogConfig {
    pub fn apply(&self, data: u16) -> f32 {
        // Select range mapping function
        let mut value = if self.fullrange {
            self.analog_map_full(data)
        } else {
            self.analog_map_half(data)
        };

        // Reverse the value if needed
        if self.reverse {
            value = -value
        }

        // Apply rates and return
        self.rates.apply(value)
    }

    /// Full-range, maps the input range to the output range (-1, 1).
    /// Typically used for roll, pitch and yaw commands.
    fn analog_map_full(&self, data: u16) -> f32 {
        
        // Center the value around 0
        let mut value = data.saturating_sub(self.in_min) as i32 - (self.in_max - self.in_min) as i32/2;

        // Apply deadband on small values
        if value.abs() < self.deadband as i32 { value = 0 }

        // Normalize and clamp the value to the range (-1, 1)
        ((2*value) as f32 / (self.in_max - self.in_min) as f32).clamp(-1., 1.)
    }

    /// Half-range, maps the input range to the output range (0, 1).
    /// Typically used for thrust commands.
    pub fn analog_map_half(&self, data: u16) -> f32 {
        
        // Shift the value down to 0
        let mut value = data.saturating_sub(self.in_min);

        // Apply deadband on small values
        if value < self.deadband { value = 0 }

        // Normalize and clamp the value to the range (0, 1)
        (value as f32 / (self.in_max - self.in_min) as f32).clamp(0., 1.)
    }

    pub fn sanity_check_rates(&self) -> Result<(), RatesError> {
        self.rates.sanity_check()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Rates {
    None,
    Standard(StandardRates),
    // More to come?
}

impl Rates {

    pub const fn new_standard(rate: f32, expo: f32, slow: f32) -> Self {
        Self::Standard(StandardRates::new(rate, expo, slow))
    }

    pub fn apply(&self, input: f32) -> f32 {
        match self {
            Self::None => input,
            Self::Standard(rates) => rates.apply(input),
        }
    }

    pub fn sanity_check(&self) -> Result<(), RatesError> {
        match self {
            Self::None => {Ok(())},
            Self::Standard(rates) => rates.sanity_check(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StandardRates {
    rate: f32,
    expo: f32,
    slow: f32,
}

impl StandardRates {

    /// Create a new set of rates. This function will panic if the rates are not within reasonable limits.
    pub const fn new(rate: f32, expo: f32, slow: f32) -> Self {
        Self {
            rate,
            expo,
            slow,
        }
    }

    /// Do an assertion on the rates constants to make sure they are within reasonable limits.
    pub fn sanity_check(&self) -> Result<(), RatesError>{

        if self.rate < 0.0 {
            return Err(RatesError::NegativeGain);
        } else if self.expo < 0.0 {
            return Err(RatesError::NegativeGain);
        } else if self.slow < 0.0 {
            return Err(RatesError::NegativeGain);
        } else if self.apply(1.0) > 10.0 {
            return Err(RatesError::ExtremeOutput);
        }

        Ok(())
    }

    /// Apply the rates to the input command
    pub fn apply(&self, input: f32) -> f32 {
        ((1. + 0.01 * self.expo * (input * input - 1.0)) * input)
        * (self.rate + (input.abs() * self.rate * self.slow * 0.01))
    }
}

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq)]
pub enum RatesError {
    NegativeGain,
    ExtremeOutput,
}