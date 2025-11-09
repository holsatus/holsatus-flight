use crate::types::status::PidTerms;

use super::{Lowpass, NthOrderLowpass, SlewRate};

#[allow(unused_imports)]
use num_traits::Float;

/// The `RatePid` type is different from the [`super::pid::Pid`] controller
/// since this is specifically designed for the innermost control loop of a
/// quadcopter.
pub struct RatePid {
    // Gains
    kp: f32,
    ki: f32,
    kd: f32,

    // States
    integral: f32,
    prev_meas: f32,
    prev_ref: f32,

    // Terms for logging
    terms: PidTerms,

    // Configuration
    dt: f32,
    max_integral: f32,
    integral_en: bool,
    lowpass: Lowpass<f32>,
    d_lowpass: NthOrderLowpass<f32, 2>,
    d_slew: Option<SlewRate<f32>>,
}

impl RatePid {
    pub fn new(kp: f32, ki: f32, kd: f32, tau: f32, dt: f32) -> Self {
        Self {
            kp,
            ki: ki,
            kd: kd * kp,
            integral: 0.0,
            prev_meas: 0.0,
            prev_ref: 0.0,
            terms: PidTerms::default(),
            integral_en: true,
            dt,
            max_integral: 1.0,
            lowpass: Lowpass::new(tau * 10., dt),
            d_lowpass: NthOrderLowpass::new(tau, dt),
            d_slew: None,
        }
    }

    pub fn set_ts(&mut self, ts: f32) {
        self.dt = ts;
        self.d_lowpass.set_dt(ts);
        if let Some(slew) = &mut self.d_slew {
            slew.set_dt(ts);
        }
    }

    pub fn slew(mut self, slew: f32) -> Self {
        self.d_slew = Some(SlewRate::new(slew, self.dt));
        self
    }

    pub fn update(&mut self, mut reference: f32, measurement: f32, prediction: f32) -> f32 {
        // Calculate proportional error
        let proportional = self.kp * (reference - measurement);

        // Optionally slew-rate limit the reference signal to prevent very quick
        // changes from being clipped by derivative controller.
        if let Some(d_slew) = self.d_slew.as_mut() {
            reference = d_slew.update(reference);
        }

        let d_gain = self.kd / self.dt;

        // Calculate derivative of reference
        let ref_derivative = d_gain * (reference - self.prev_ref);
        self.prev_ref = reference;

        // Take derivatives of measurement (negative because d(ref - meas)/dt)
        let measurement_lp = self.d_lowpass.update(measurement);
        let meas_derivative = d_gain * (self.prev_meas - measurement_lp);
        self.prev_meas = measurement_lp;

        // Limit integrator to only be actuve during "calm" moves
        const CALM_THRESHOLD: f32 = 1.0;

        // Calculate error integral (if reference is small) This ensures we do
        // not carry over integration which occured for large references
        // (velocities) where air resistance and other strange dynamics likely
        // played a larger role than at a low reference.
        if self.integral_en 
        && measurement_lp.abs() < CALM_THRESHOLD 
        && reference.abs() < CALM_THRESHOLD {
            let model_error = prediction - measurement;
            let damping = 1.0 + self.lowpass.update(meas_derivative.abs());
            self.integral += self.ki * model_error * self.dt / damping;
            self.integral = self.integral.clamp(-self.max_integral, self.max_integral);

        }

        self.terms = PidTerms {
            p_out: proportional,
            i_out: self.integral,
            dr_out: ref_derivative,
            dm_out: meas_derivative,
        };

        // Add all terms together
        proportional + self.integral + ref_derivative + meas_derivative
    }

    pub fn reset_integral(&mut self) {
        self.integral = 0.0;
    }

    pub fn enable_integral(&mut self, enable: bool) {
        self.integral_en = enable;
    }

    pub fn enable_reset_integral(&mut self, enable: bool) {
        self.enable_integral(enable);
        self.reset_integral();
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd * kp;
    }

    pub fn get_terms(&self) -> &PidTerms {
        &self.terms
    }
}
