pub mod angle_pid;
pub mod linear_lut;
pub mod motor_lin;
pub mod rate_pid;

use core::array::from_fn;
use num_traits::Float;
use serde::{Deserialize, Serialize};

pub trait SisoFilter {
    type Type;
    fn update(&mut self, input: Self::Type) -> Self::Type;
    fn as_dyn(&mut self) -> &mut dyn SisoFilter<Type = Self::Type>
    where
        Self: Sized,
    {
        self as &mut dyn SisoFilter<Type = Self::Type>
    }
}

pub trait FilterStructure
where
    Self: SisoFilter + Clone,
{
    // Convert the filter to be a `N`'th order filter of the same type.
    fn order<const N: usize>(self) -> Order<Self, N> {
        Order {
            filters: from_fn(|_| self.clone()),
        }
    }

    /// Chain two SISO filters together. When updating the chain, the input is
    /// first passed through the `self` filter and then the `other` filter.
    fn chain<F2: SisoFilter<Type = Self::Type>>(self, other: F2) -> Chain<Self::Type, Self, F2> {
        Chain {
            filter1: self,
            filter2: other,
        }
    }
}

impl<T: SisoFilter + Clone> FilterStructure for T {}

#[derive(Clone)]
pub struct Chain<F, F1: SisoFilter<Type = F>, F2: SisoFilter<Type = F>> {
    filter1: F1,
    filter2: F2,
}

impl<F: Clone, F1: SisoFilter<Type = F>, F2: SisoFilter<Type = F>> SisoFilter for Chain<F, F1, F2> {
    type Type = F;
    fn update(&mut self, input: Self::Type) -> Self::Type {
        self.filter2.update(self.filter1.update(input))
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Lowpass<T: Float> {
    tau: T,
    dt: T,
    y: T,
    alpha0: T,
    alpha1: T,
}

impl<T: Float> Lowpass<T> {
    pub fn new(tau: T, dt: T) -> Self {
        let alpha0 = dt / (tau + dt);
        let alpha1 = T::one() - alpha0;
        Self {
            tau,
            dt,
            y: T::zero(),
            alpha0,
            alpha1,
        }
    }

    pub fn set_dt(&mut self, dt: T) {
        self.dt = dt;
        self.alpha0 = dt / (self.tau + dt);
        self.alpha1 = T::one() - self.alpha0;
    }

    pub fn set_tau(&mut self, tau: T) {
        self.tau = tau;
        self.alpha0 = self.dt / (tau + self.dt);
        self.alpha1 = T::one() - self.alpha0;
    }

    pub fn update(&mut self, x: T) -> T {
        self.y = self.alpha0 * x + self.alpha1 * self.y;
        self.y
    }
}

impl<F: Float> SisoFilter for Lowpass<F> {
    type Type = F;
    fn update(&mut self, input: Self::Type) -> Self::Type {
        self.update(input)
    }
}

impl<F: SisoFilter, const N: usize> SisoFilter for Order<F, N> {
    type Type = F::Type;
    fn update(&mut self, input: Self::Type) -> Self::Type {
        self.update(input)
    }
}

#[derive(Clone)]
pub struct Order<F: SisoFilter, const N: usize> {
    filters: [F; N],
}

impl<F: SisoFilter, const N: usize> Order<F, N> {
    pub fn update(&mut self, x: F::Type) -> F::Type {
        self.filters.iter_mut().fold(x, |y, f| f.update(y))
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NthOrderLowpass<T: Float, const N: usize> {
    filters: [Lowpass<T>; N],
}

impl<T: Float, const N: usize> NthOrderLowpass<T, N> {
    pub fn new(tau: T, dt: T) -> Self {
        Self {
            filters: from_fn(|_| Lowpass::new(tau, dt)),
        }
    }

    pub fn set_dt(&mut self, dt: T) {
        for filter in self.filters.iter_mut() {
            filter.set_dt(dt);
        }
    }

    pub fn configure(&mut self, tau: T) {
        for filter in self.filters.iter_mut() {
            filter.set_tau(tau);
        }
    }

    pub fn update(&mut self, x: T) -> T {
        let mut y = x;
        for filter in self.filters.iter_mut() {
            y = filter.update(y);
        }
        y
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct Highpass<T: Float> {
    tau: T,
    dt: T,
    y: T,
    x_prev: T,
    alpha: T,
}

impl<T: Float> Highpass<T> {
    pub fn new(tau: T, dt: T) -> Self {
        let alpha = tau / (tau + dt);
        Self {
            tau,
            dt,
            y: T::zero(),
            x_prev: T::zero(),
            alpha,
        }
    }

    pub fn set_dt(&mut self, dt: T) {
        self.dt = dt;
        self.alpha = self.tau / (self.tau + dt);
    }

    pub fn set_tau(&mut self, tau: T) {
        self.tau = tau;
        self.alpha = tau / (tau + self.dt);
    }

    pub fn update(&mut self, x: T) -> T {
        self.y = self.alpha * self.y + self.alpha * (x - self.x_prev);
        self.x_prev = x;
        self.y
    }
}

/// Basic complementary filter implementation. This filter takes in two signals,
/// one with a low frequency component which we want to preserve, and one with
/// a high frequency component.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Complementary<T: Float> {
    lp_filter: Lowpass<T>,
    hp_filter: Highpass<T>,
}

impl<T: Float> Complementary<T> {
    /// Create a new `Complementary` filter with the cross-over frequency `tau`
    pub fn new(tau: T, dt: T) -> Self {
        Self {
            lp_filter: Lowpass::new(tau, dt),
            hp_filter: Highpass::new(tau, dt),
        }
    }

    /// Update the filter with new signal vales. The `low_frequency` argument is the
    /// signal with a low-frequency component we want to preserve. This could for
    /// example be a noisy sensor, with no bias or drift. The `high_frequency` argument
    /// is the complementary signal. This could be a signal which has short term
    /// "high frequency" behevior which is desireable, but not necessarily good long-term
    /// "low-freqnency" characteristics. This could be an integrated signal or a
    /// feed-forward estimated value. The output combines the best of both signals.
    pub fn update(&mut self, low_frequency: T, high_frequency: T) -> T {
        let lf = self.lp_filter.update(low_frequency);
        let hf = self.hp_filter.update(high_frequency);

        lf + hf
    }

    /// Set the cross-over frequency of the filter with a time-constant `tau`
    pub fn set_dt(&mut self, dt: T) {
        self.lp_filter.set_dt(dt);
        self.hp_filter.set_dt(dt);
    }

    /// Set the cross-over frequency of the filter with a time-constant `tau`
    pub fn set_tau(&mut self, tau: T) {
        self.lp_filter.set_tau(tau);
        self.hp_filter.set_tau(tau);
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RatesType {
    Actual,
    Kiss,
}

impl<T: Float> Default for ExpoRates<T> {
    fn default() -> Self {
        Self {
            rates_type: RatesType::Actual,
            expo: T::from(0.2).unwrap(),
            max_rates: T::from(31.41).unwrap(),
            center_sens: T::from(0.4).unwrap(),
        }
    }
}

impl<T: Float> SisoFilter for ExpoRates<T> {
    type Type = T;
    fn update(&mut self, input: Self::Type) -> Self::Type {
        self.apply(input)
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExpoRates<T: Float> {
    rates_type: RatesType,
    expo: T,
    max_rates: T,
    center_sens: T,
}

impl<T: Float> ExpoRates<T> {
    pub fn new(rates_type: RatesType, expo: T, max_rates: T, center_sens: T) -> Self {
        Self {
            rates_type,
            expo,
            max_rates,
            center_sens,
        }
    }

    pub fn apply(&self, rc_command: T) -> T {
        match self.rates_type {
            RatesType::Actual => self.apply_actual_rates(rc_command),
            RatesType::Kiss => self.apply_kiss_rates(rc_command),
        }
    }

    fn apply_actual_rates(&self, rc_command: T) -> T {
        let rc_command_abs = rc_command.abs();
        let expof =
            rc_command_abs * (rc_command.powi(5) * self.expo + rc_command * (T::one() - self.expo));

        let center_sensitivity = self.center_sens * self.max_rates;
        let stick_movement = T::zero().max(self.max_rates - center_sensitivity);
        rc_command * center_sensitivity + stick_movement * expof
    }

    fn apply_kiss_rates(&self, rc_command: T) -> T {
        let kiss_rpy_use_rates = T::one()
            / (T::one()
                - rc_command.abs() * self.max_rates.max(T::from(0.01).unwrap()).min(T::one()));
        let kiss_rc_command =
            (rc_command.powi(3) * self.expo + rc_command * (T::one() - self.expo)) * self.max_rates;
        kiss_rpy_use_rates * kiss_rc_command
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Linear<T: Float + Serialize> {
    factor: T,
    offset: T,
}

impl From<Linear<f32>> for crate::rc_mapping::analog::Rates {
    fn from(linear: Linear<f32>) -> Self {
        crate::rc_mapping::analog::Rates::Linear(linear)
    }
}

impl Linear<f32> {
    pub const fn const_default() -> Self {
        Self {
            factor: 1.0,
            offset: 0.0,
        }
    }
}

impl Linear<f32> {
    pub const fn new(in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> Self {
        let mut new = Self::const_default();
        new.set(in_min, in_max, out_min, out_max);
        new
    }

    // Change the mapping of the linear function
    pub const fn set_params(&mut self, in_min: f32, in_max: f32, out_min: f32, out_max: f32) {
        self.factor = (out_max - out_min) / (in_max - in_min);
        self.offset = out_min - in_min * (out_max - out_min) / (in_max - in_min);
    }

    pub const fn set(&mut self, in_min: f32, in_max: f32, out_min: f32, out_max: f32) {
        self.factor = (out_max - out_min) / (in_max - in_min);
        self.offset = out_min - in_min * (out_max - out_min) / (in_max - in_min);
    }

    pub const fn map(&self, sig: f32) -> f32 {
        sig * self.factor + self.offset
    }

    /// Returns (factor, offset) or for a*x+b, (a, b)
    pub const fn params(&self) -> (f32, f32) {
        (self.factor, self.offset)
    }

    /// Returns (factor, offset) or for a*x+b, (a, b)
    pub const fn params_mut(&mut self) -> (&mut f32, &mut f32) {
        (&mut self.factor, &mut self.offset)
    }
}

impl<T: Float, const N: usize> SisoFilter for MovingAverage<T, N> {
    type Type = T;
    fn update(&mut self, input: Self::Type) -> Self::Type {
        self.update(input)
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MovingAverage<T: Float, const N: usize> {
    array: [T; N],
    div: T,
    head: usize,
}

impl<T: Float, const N: usize> MovingAverage<T, N> {
    pub fn new() -> Self {
        Self {
            array: [T::zero(); N],
            div: T::from(N).unwrap(),
            head: 0,
        }
    }

    pub fn new_with_value(value: T) -> Self {
        Self {
            array: [value; N],
            div: T::from(N).unwrap(),
            head: 0,
        }
    }

    pub fn update(&mut self, value: T) -> T {
        if let Some(val) = self.array.get_mut(self.head) {
            *val = value;
            self.head += 1
        } else {
            self.array[0] = value;
            self.head = 0;
        }

        self.array.iter().fold(T::zero(), |sum, x| sum + *x) / self.div
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlewRate<T: Float> {
    rate_limit: T,
    signal: T,
    dt: T,
}

impl<T: Float> SlewRate<T> {
    /// Construct a new rate limiter
    ///
    /// The `rate_limit` is the maximum, and (negative) minimum
    /// the filter will allow to be changed per unit time.
    pub fn new(rate_limit: T, dt: T) -> Self {
        assert!(rate_limit.is_sign_positive());
        Self {
            rate_limit,
            signal: T::zero(),
            dt,
        }
    }

    pub fn update(&mut self, input: T) -> T {
        let change = (input - self.signal)
            .max(-self.rate_limit * self.dt)
            .min(self.rate_limit * self.dt);

        self.signal = self.signal + change;
        self.signal
    }

    pub fn set_dt(&mut self, dt: T) {
        self.dt = dt;
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct BiquadFilter<T: Float> {
    params: BiquadParams<T>,
    x: [T; 3],
    y: [T; 3],
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct BiquadParams<T: Float> {
    a: [T; 3],
    b: [T; 3],
}

impl<T: Float> BiquadFilter<T> {
    pub fn new(a: [T; 3], b: [T; 3]) -> Self {
        Self {
            params: BiquadParams { a, b },
            x: [T::zero(); 3],
            y: [T::zero(); 3],
        }
    }

    pub fn update(&mut self, input: T) -> T {
        let a = self.params.a;
        let b = self.params.b;
        let x = &mut self.x;
        let y = &mut self.y;

        x[2] = x[1];
        x[1] = x[0];
        x[0] = input;

        y[2] = y[1];
        y[1] = y[0];

        let output = x[0] * b[0] + x[1] * b[1] + x[2] * b[2] - y[1] * a[1] - y[2] * a[2];

        y[0] = output / a[0];
        y[0]
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NotchFilter<T: Float> {
    fc: T,
    fb: T,
    dt: T,
    biquad: BiquadFilter<T>,
}

impl<T: Float> NotchFilter<T> {
    pub fn new(fc: T, fb: T, dt: T) -> Self {
        let params = Self::calc_params(fc, fb, dt);
        Self {
            fc,
            fb,
            dt,
            biquad: BiquadFilter::new(params.a, params.b),
        }
    }

    fn calc_params(fc: T, fb: T, dt: T) -> BiquadParams<T> {
        let pi = T::from(core::f64::consts::PI).unwrap();
        let two = T::from(2.0).unwrap();
        let four = T::from(4.0).unwrap();

        let wc: T = two / dt * (fc * two * pi * dt / two).tan();
        let wb: T = fb * two * pi;

        let b_temp = wc * wc * dt * dt;

        let b = [b_temp + four, (b_temp - four) * two, b_temp + four];

        let a = [b[0] + two * wb * dt, b[1], b[2] - two * wb * dt];

        BiquadParams { a, b }
    }

    pub fn set_freq_center(&mut self, fc: T) {
        self.fc = fc;
        let params = Self::calc_params(self.fc, self.fb, self.dt);
        self.biquad.params.a = params.a;
        self.biquad.params.b = params.b;
    }

    pub fn set_freq_bandwidth(&mut self, fb: T) {
        self.fb = fb;
        let params = Self::calc_params(self.fc, self.fb, self.dt);
        self.biquad.params.a = params.a;
        self.biquad.params.b = params.b;
    }

    pub fn update(&mut self, input: T) -> T {
        self.biquad.update(input)
    }
}
