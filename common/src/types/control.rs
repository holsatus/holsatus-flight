use embassy_time::Instant;
use serde::{Deserialize, Serialize};

/// Analog RC control commands, e.g. a radio transmitter. In order,
/// the numbers represent, roll, pitch, yaw and throttle and 4 aux channels.
#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcAnalog {
    pub timestamp_us: u64,
    pub values: [f32; 8],
}

impl RcAnalog {
    pub fn new(roll: f32, pitch: f32, yaw: f32, thrust: f32) -> Self {
        RcAnalog {
            timestamp_us: Instant::now().as_micros(),
            values: [roll, pitch, yaw, thrust, 0., 0., 0., 0.],
        }
    }

    pub fn roll_pitch_yaw(&self) -> [f32; 3] {
        [self.values[0], self.values[1], self.values[2]]
    }

    pub fn throttle(&self) -> f32 {
        self.values[3]
    }

    pub fn aux(&self) -> [f32; 4] {
        [
            self.values[4],
            self.values[5],
            self.values[6],
            self.values[7],
        ]
    }
}
