use nalgebra::{SVector, Unit, UnitQuaternion};

/// A filter that dynamically achieves 'First-Order Hold'-like behavior.
/// Commonly used to upsample signals to avoid stair stepping, useful for taking derivatives.
/// The implementation assumes that samples are both provided and requested at fairly steady rates.
#[derive(Debug)]
pub struct RampSmoother<T> {
    period: f32,
    inv_period: f32,
    count: usize,
    prev_sample: T,
    curr_sample: T,
}

impl<T> RampSmoother<T>
where
    T: Interpolate,
{
    const ALPHA: f32 = 0.9;

    pub fn new(initial: T) -> Self {
        RampSmoother {
            period: 10.0,
            inv_period: 1. / 10.,
            count: 10,
            prev_sample: initial,
            curr_sample: initial,
        }
    }

    /// Interpolate between the current and previous sample values
    fn interpolate(&self) -> T {
        let ratio = self.count as f32 * self.inv_period;
        let t = if ratio < 1.0 { ratio } else { 1.0 };
        self.prev_sample.interpolate(&self.curr_sample, t)
    }

    /// Update the filter with a new input value
    pub fn add_sample(&mut self, sample: T) {
        self.prev_sample = self.interpolate();
        self.curr_sample = sample;
        let new_period = self.period * Self::ALPHA + self.count as f32 * (1.0 - Self::ALPHA);
        self.period = if new_period > 1.0 { new_period } else { 1.0 };
        self.inv_period = 1.0 / self.period;
        self.count = 0;
    }

    /// Get the next output value of the filter
    /// Note, this updates the state of the filter
    pub fn get(&mut self) -> T {
        self.count += 1;
        self.interpolate()
    }
}

pub trait Interpolate: Copy {
    fn interpolate(&self, other: &Self, alpha: f32) -> Self;
}

impl<const N: usize> Interpolate for [f32; N] {
    fn interpolate(&self, other: &Self, alpha: f32) -> Self {
        core::array::from_fn(|index| self[index] + (other[index] - self[index]) * alpha)
    }
}

impl<const N: usize> Interpolate for SVector<f32, N> {
    fn interpolate(&self, other: &Self, alpha: f32) -> Self {
        self + (other - self) * alpha
    }
}

impl Interpolate for UnitQuaternion<f32> {
    fn interpolate(&self, other: &Self, alpha: f32) -> Self {
        Unit::new_normalize(self.lerp(other, alpha))
    }
}
