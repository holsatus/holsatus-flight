#[cfg(not(feature = "arch-std"))]
use num_traits::Float;
use serde::{Deserialize, Serialize};

pub use crate::filters::Linear;

use super::Binding;

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Axis {

    // Primary control axes
    Roll,
    Pitch,
    Yaw,
    Throt,

    // Aux channels can be used for whatever
    Aux1,
    Aux2,
    Aux3,
    Aux4,
    Aux5,
    Aux6,
    Aux7,
    Aux8,
    Aux9,

    // Maybe excessive, but might as well
    Aux10,
    Aux11,
    Aux12,
    Aux13,
    Aux14,
    Aux15,
    Aux16,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogBind {
    pub axis: Axis,
    pub in_min: u16,
    pub in_max: u16,
    pub deadband: u8,
    pub fullrange: bool,
    pub reverse: bool,
    pub rates: Rates,
}

impl From<AnalogBind> for Binding {
    fn from(cfg: AnalogBind) -> Self {
        Binding::Analog(cfg)
    }
}

impl AnalogBind {
    /// Full-range, maps the input range to the output range (-1, 1). Typically
    /// used for roll, pitch and yaw commands.
    pub fn map_full_range(&self, data: u16) -> f32 {
        // Center the value around 0
        let mut value =
            data.saturating_sub(self.in_min + 1) as i32 - (self.in_max - self.in_min) as i32 / 2;

        // Apply deadband on small values
        if value.abs() < self.deadband as i32 {
            value = 0
        }

        // Normalize and clamp the value to the range (-1, 1)
        let value = ((2 * value) as f32 / (self.in_max - self.in_min - 1) as f32)
            .clamp(-1., 1.);

        if self.reverse {
            -value
        } else {
            value
        }
    }

    /// Half-range, maps the input range to the output range (0, 1). Typically
    /// used for thrust commands.
    pub fn map_half_range(&self, data: u16) -> f32 {
        // Shift the value down to 0
        let mut value = data.saturating_sub(self.in_min);

        // Apply deadband on small values
        if value < self.deadband as u16 {
            value = 0
        }

        // Normalize and clamp the value to the range (0, 1)
        let value = (value as f32 / (self.in_max - self.in_min) as f32)
            .clamp(0., 1.);

        // Reverse the value if needed
        if self.reverse {
            1.0 - value
        } else {
            value
        }
    }
}

impl Configuration for AnalogBind {
    fn sanity_check(&self) -> Result<(), ConfigurationError> {
        if self.in_min >= self.in_max {
            return Err(ConfigurationError::InvalidRange);
        }

        if self.deadband as u16 > self.in_max - self.in_min {
            return Err(ConfigurationError::InvalidDeadband);
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rates {
    None,
    /// Actual rates provide an "expo" response
    Actual(Actual),
    /// Affine linear rates are of the form a * x + b
    Linear(Linear<f32>),
}

pub trait RatesTrait {
    fn apply(&self, input: f32) -> f32;
}

impl RatesTrait for Rates {
    fn apply(&self, input: f32) -> f32 {
        match self {
            Rates::None => input,
            Rates::Actual(actual) => actual.apply(input),
            Rates::Linear(linear) => linear.map(input),
        }
    }
}

impl RatesTrait for Linear<f32> {
    fn apply(&self, input: f32) -> f32 {
        self.map(input)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Actual {

    /// Sensitivity (rate of change) at maximum deflection
    pub rate: f32,

    /// Expo pushes the effect of `rate` further from the center
    pub expo: f32,

    /// Sensitivity (rate of change) of input values around center
    pub cent: f32,
}

impl From<Actual> for Rates {
    fn from(actual: Actual) -> Self {
        Rates::Actual(actual)
    }
}

impl Configuration for Rates {
    fn sanity_check(&self) -> Result<(), ConfigurationError> {
        match self {
            Rates::None => {}
            Rates::Actual(actual) => {
                if actual.cent > actual.rate {
                    return Err(ConfigurationError::InvalidRates);
                }
            }
            Rates::Linear(linear) => {
                if linear.params().0 <= 0. {
                    return Err(ConfigurationError::InvalidRates);
                }
            }
        }

        Ok(())
    }
}

impl Actual {
    pub const fn const_default() -> Self {
        Self {
            rate: 20.,
            expo: 0.5,
            cent: 5.0,
        }
    }
}

impl RatesTrait for Actual {
    fn apply(&self, val: f32) -> f32 {
        self.cent * val + (self.rate - self.cent)
        * (val.abs() * (val.powi(5) * self.expo + val * (1.0 - self.expo)))
    }
}



pub enum ConfigurationError {
    InvalidRange,
    InvalidDeadband,
    InvalidRates,
}

pub trait Configuration {
    fn sanity_check(&self) -> Result<(), ConfigurationError>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxMapError {
    RatesNegativeGain,
    RatesExtremeOutput,
    MinMaxInverted,
}
