#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(mav_param::Enum)]
#[repr(u8)]
pub enum Rates {
    /// No rates are applied. The output is the input
    Identity,
    /// Actual rates provide an "expo" response
    Actual(Actual),
    /// Affine linear rates are of the form a * x + b
    Linear(Linear),
}

crate::const_default!(
    Rates => Rates::Identity
);

impl Rates {
    pub fn apply(&self, input: f32) -> f32 {
        match self {
            Rates::Identity => input,
            Rates::Actual(actual) => actual.apply(input),
            Rates::Linear(linear) => linear.apply(input),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(mav_param::Tree)]
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

crate::const_default!(
    Actual => {
        rate: 50.,
        expo: 0.5,
        cent: 5.0,
    }
);

impl Actual {
    pub fn apply(&self, val: f32) -> f32 {
        #[allow(unused)]
        #[cfg(not(feature = "arch-std"))]
        use num_traits::Float as _;
        self.cent * val
            + (self.rate - self.cent)
                * (val.abs() * (val.powi(5) * self.expo + val * (1.0 - self.expo)))
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(mav_param::Tree)]
pub struct Linear {
    pub fact: f32,
    pub offs: f32,
}

crate::const_default!(
    Linear => {
        fact: 1.,
        offs: 0.,
    }
);

impl Linear {
    pub const fn new(in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> Self {
        let mut new = Self::const_default();
        new.set(in_min, in_max, out_min, out_max);
        new
    }

    // Change the mapping of the linear function
    pub const fn set_params(&mut self, in_min: f32, in_max: f32, out_min: f32, out_max: f32) {
        self.fact = (out_max - out_min) / (in_max - in_min);
        self.offs = out_min - in_min * (out_max - out_min) / (in_max - in_min);
    }

    pub const fn set(&mut self, in_min: f32, in_max: f32, out_min: f32, out_max: f32) {
        self.fact = (out_max - out_min) / (in_max - in_min);
        self.offs = out_min - in_min * (out_max - out_min) / (in_max - in_min);
    }

    pub const fn apply(&self, val: f32) -> f32 {
        val * self.fact + self.offs
    }
}
