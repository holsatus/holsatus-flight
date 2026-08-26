/// A filter that dynamically achieves 'First-Order Hold'-like behavior.
/// Commonly used to upsample signals to avoid stair stepping.
/// The implementation assumes that samples are both provided and requested at a fairly steady rate.
///
#[derive(Debug)]
pub struct FohSmoother {
    period: f32,
    count: usize,
    prev_sample: f32,
    curr_sample: f32,
}

impl FohSmoother {
    const ALPHA: f32 = 0.9;

    pub fn new(initial: f32) -> Self {
        FohSmoother {
            period: 10.0,
            count: 10,
            prev_sample: initial,
            curr_sample: initial,
        }
    }

    /// Interpolate between the current and previous sample values
    fn interpolate(&self) -> f32 {
        let ratio = self.count as f32 / self.period;
        let t = if ratio < 1.0 { ratio } else { 1.0 };
        self.prev_sample + (self.curr_sample - self.prev_sample) * t
    }

    /// Update the filter with a new input value
    pub fn add_sample(&mut self, sample: f32) {
        self.prev_sample = self.interpolate();
        self.curr_sample = sample;
        let new_period = self.period * Self::ALPHA + self.count as f32 * (1.0 - Self::ALPHA);
        self.period = if new_period > 1.0 { new_period } else { 1.0 };
        self.count = 0;
    }

    /// Get the output value of the filter
    /// Note, this updates the state of the filter
    pub fn get(&mut self) -> f32 {
        self.count += 1;
        self.interpolate()
    }
}
