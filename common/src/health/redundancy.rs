use embassy_time::{Duration, Instant};
use heapless::Deque;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Sensor is in an uninitialized state
    Uninit,
    /// Sensor is active and transmitting at a normal rate
    Active,
    /// Sensor is idle and transmitting at a reduced rate
    Idle(u8),
    /// Sensor is inactive and not transmitting
    Stopped,
}

#[derive(Clone, Debug)]
pub struct SensorEvaluator<const N: usize> {
    buffer: Deque<Vector3<f32>, N>,
    stall_timeout: Duration,
    variance_cutoff: f32,
    variance: Option<Vector3<f32>>,
    pub last_reading: Option<Instant>,
    pub condition: Condition,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Condition {
    Degraded(Failure),
    Unknown,
    Good,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Failure {
    /// Sensor has stopped sending for 50 ms (default)
    Stalled,
    /// Values are not changing (zero variance)
    Stuck,
    /// Sensor is too noisy (high variance)
    Noisy,
    /// Sensor is returning invalid values
    Invalid,
}

impl<const N: usize> SensorEvaluator<N> {
    pub fn new() -> Self {
        SensorEvaluator {
            buffer: Deque::new(),
            stall_timeout: Duration::from_millis(25),
            variance_cutoff: 1e-9,
            last_reading: None,
            variance: None,
            condition: Condition::Unknown,
        }
    }

    /// Detect whether any of the sensors have stalled and mark them as `SensorFailure::Stalled`.
    pub fn add_sample(&mut self, sample: impl Into<Vector3<f32>>) {
        if self.buffer.is_full() {
            _ = self.buffer.pop_back();
        }
        _ = self.buffer.push_front(sample.into());
        self.last_reading = Some(Instant::now());
        self.variance = None;
    }

    pub fn compute_variance(&self) -> Vector3<f32> {
        let mean = self.buffer.iter().sum::<Vector3<f32>>() / N as f32;
        self.buffer
            .iter()
            .map(|x| (x - mean).component_mul(&(x - mean)))
            .sum::<Vector3<f32>>()
            / N as f32
    }

    pub fn detect_stuck(&mut self) -> bool {
        // Compute the variance of the samples (or use cached)
        let variance = self.variance.unwrap_or_else(|| self.compute_variance());
        self.variance = Some(variance);

        // If the variance on any axis is very small, the sensor is stuck
        if variance.min() < self.variance_cutoff {
            self.condition = Condition::Degraded(Failure::Stuck);
            true
        } else {
            false
        }
    }

    pub fn detect_stall(&mut self) -> bool {
        if let Some(last_reading) = self.last_reading {
            if last_reading.elapsed() > self.stall_timeout {
                self.condition = Condition::Degraded(Failure::Stalled);
                return true;
            }
        }
        false
    }
}
