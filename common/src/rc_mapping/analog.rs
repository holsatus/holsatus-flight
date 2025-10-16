#[cfg(not(feature = "arch-std"))]
use num_traits::Float;
use serde::{Deserialize, Serialize};

pub use crate::filters::Linear;

use super::Binding;

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize, num_enum::TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
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
    
    // Maybe excessive, but might as well
    Aux9,
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
    pub flags: AnalogFlags,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogFlags(u8);

bitflags::bitflags! {
    impl AnalogFlags: u8 {
        const FULLRANGE = 1 << 0;
        const REVERSE = 1 << 0;
    }
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
        let value = ((2 * value) as f32 / (self.in_max - self.in_min - 1) as f32).clamp(-1., 1.);

        if self.flags.contains(AnalogFlags::REVERSE) {
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
        let value = (value as f32 / (self.in_max - self.in_min) as f32).clamp(0., 1.);

        // Reverse the value if needed
        if self.flags.contains(AnalogFlags::REVERSE) {
            1.0 - value
        } else {
            value
        }
    }
}
