use num_traits::real::Real;

pub struct Statistics<T> {
    initialized: bool,
    alpha: T,
    mean: T,
    variance: T,
}

impl <T> Statistics<T>
where 
T: Real,
{
    /// Creates a new IIR statistics filter.
    /// `alpha` is the smoothing factor, typically a small value (e.g., 0.01).
    /// A smaller alpha means a "slower" filter that remembers more history.
    pub fn new(alpha: T) -> Self {
        if !(alpha > T::zero() && alpha <= T::one()) {
            panic!("Alpha must be in the range (0.0, 1.0]");
        }
        Self {
            alpha,
            mean: T::zero(),
            variance: T::zero(),
            initialized: false,
        }
    }

    /// Adds a single sample to the IIR filter
    pub fn add_sample(&mut self, sample: T) {
        if !self.initialized {
            // First sample: Initialize mean to this sample, variance is 0
            self.mean = sample;
            self.variance = T::zero();
            self.initialized = true;
        } else {
            // Welford's IIR update algorithm
            let delta = sample - self.mean;
            
            // Update mean
            // M_n = M_{n-1} + alpha * delta
            self.mean = self.mean + delta * self.alpha; 
            
            // Update variance
            // V_n = (1 - alpha) * (V_{n-1} + alpha * delta^2)
            self.variance = (self.variance + delta * delta * self.alpha) * (T::one() - self.alpha);
        }
    }

    /// Returns the current IIR mean and variance
    pub fn mean_var(&self) -> (T, T) {
        (self.mean, self.variance)
    }

    /// Returns the current IIR standard deviation
    pub fn std_dev(&self) -> T {
        self.variance.sqrt()
    }
}
