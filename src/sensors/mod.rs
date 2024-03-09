pub mod gnss;
pub mod imu;

use embassy_time::{Duration, Instant};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SensorCondition {
    Unknown,
    Good,
    Degraded(SensorFailure),
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SensorState {
    pub last_reading: Option<Instant>,
    pub condition: SensorCondition,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SensorSendMode {
    /// Sensor is active and transmitting at a normal rate
    Active,
    /// Sensor is idle and transmitting at a reduced rate
    Idle,
    /// Sensor is inactive and not transmitting
    Stopped,
}

impl Default for SensorState {
    fn default() -> Self {
        Self {
            last_reading: Default::default(),
            condition: SensorCondition::Unknown,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SensorFailure {
    /// Sensor has stopped sending for 50 ms (default)
    Stalled,
    /// Values are not changing (zero variance)
    Frozen,
    /// Sensor is noisy (high variance)
    Noisy,
    /// Sensor is returning invalid values
    Invalid,
}

#[derive(Copy, Clone, Debug)]
pub struct SensorRedundancy<const N: usize> {
    active_id: Option<u8>,
    sensors: [SensorState; N],
    stall_timeout: Duration,
}

impl<const N: usize> Default for SensorRedundancy<N> {
    fn default() -> Self {
        SensorRedundancy {
            active_id: Some(0),
            sensors: [SensorState::default(); N],
            stall_timeout: Duration::from_millis(50),
        }
    }
}

impl<const N: usize> SensorRedundancy<N> {
    pub const fn const_new() -> Self {
        SensorRedundancy {
            active_id: Some(0),
            sensors: [SensorState {
                last_reading: None,
                condition: SensorCondition::Unknown,
            }; N],
            stall_timeout: Duration::from_millis(50),
        }
    }

    pub fn active_id(&self) -> Option<u8> {
        self.active_id
    }

    pub fn lower_state(&mut self) {
        if let Some(id) = self.active_id {
            if id < N as u8 {
                self.active_id = Some(id + 1);
            } else {
                self.active_id = None;
            }
        }
    }

    pub fn get(&self, id: u8) -> SensorState {
        self.sensors[id as usize]
    }

    pub fn get_mut(&mut self, id: u8) -> &mut SensorState {
        &mut self.sensors[id as usize]
    }

    pub fn set_cond(&mut self, id: u8, cond: SensorCondition) {
        self.get_mut(id).condition = cond;
    }

    /// Returns `true` if the sensor is in a `SensorCondition::Good` condition
    pub fn is_active(&self, id: u8) -> bool {
        self.active_id == Some(id)
    }

    /// Returns `true` if the sensor is in a `SensorCondition::Unknown` condition
    pub fn is_unknown(&self, id: u8) -> bool {
        self.get(id).condition == SensorCondition::Unknown
    }

    /// Returns `true` if the sensor is in a `SensorCondition::Good` condition
    pub fn is_good(&self, id: u8) -> bool {
        self.get(id).condition == SensorCondition::Good
    }

    /// Returns `true` if the sensor is in a `SensorCondition::Degraded(_)` condition
    pub fn is_degraded(&self, id: u8) -> bool {
        if let SensorCondition::Degraded(_) = self.get(id).condition {
            true
        } else {
            false
        }
    }

    /// Detect whether any of the sensors have stalled and mark them as `SensorCondition::Degraded(SensorFailure::Stalled)`.
    pub fn detect_stall_any(&mut self) {
        for sensor in &mut self.sensors {
            if let Some(last_reading) = sensor.last_reading {
                if last_reading.elapsed() > self.stall_timeout {
                    sensor.condition = SensorCondition::Degraded(SensorFailure::Stalled);
                }
            }
        }
    }
}
