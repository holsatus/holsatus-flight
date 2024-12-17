use crate::types::status::PidTerms;

use super::{Lowpass, NthOrderLowpass, SlewRate};

#[allow(unused_imports)]
use num_traits::Float;
use serde::{Deserialize, Serialize};

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
    ts: f32,
    integral_en: bool,
    lowpass: Lowpass<f32>,
    d_lowpass: NthOrderLowpass<f32, 2>,
    d_slew: Option<SlewRate<f32>>,
}

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct RatePidCfg {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub tau: f32,
    pub slew: Option<f32>,
}

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub struct RatePidCfg3D {
    pub x: RatePidCfg,
    pub y: RatePidCfg,
    pub z: RatePidCfg,
}

impl Default for RatePidCfg3D {
    fn default() -> Self {
        Self {
            x: RatePidCfg {
                kp: 0.02,
                ki: 10.,
                kd: 0.05,
                tau: 0.002,
                slew: Some(400.),
            },
            y: RatePidCfg {
                kp: 0.02,
                ki: 10.,
                kd: 0.05,
                tau: 0.002,
                slew: Some(400.),
            },
            z: RatePidCfg {
                kp: 0.01,
                ki: 10.,
                kd: 0.1,
                tau: 0.002,
                slew: Some(400.),
            },
        }
    }
}

impl RatePid {
    pub fn new(kp: f32, ki: f32, kd: f32, ts: f32, tau: f32) -> Self {
        Self {
            kp,
            ki: ki * kp,
            kd: kd * kp,
            integral: 0.0,
            prev_meas: 0.0,
            prev_ref: 0.0,
            terms: PidTerms::default(),
            integral_en: true,
            ts,
            lowpass: Lowpass::new(tau * 5., ts),
            d_lowpass: NthOrderLowpass::new(tau, ts),
            d_slew: None,
        }
    }

    pub fn new_from_cfg(cfg: RatePidCfg, ts: f32) -> Self {
        let mut pid = Self::new(cfg.kp, cfg.ki, cfg.kd, ts, cfg.tau);
        if let Some(slew) = cfg.slew {
            pid = pid.slew(slew);
        }
        pid
    }

    pub fn set_config(&mut self, cfg: RatePidCfg) {
        self.kp = cfg.kp;
        self.ki = cfg.ki * cfg.kp;
        self.kd = cfg.kd * cfg.kp;
        self.lowpass = Lowpass::new(cfg.tau * 5., self.ts);
        self.d_lowpass = NthOrderLowpass::new(cfg.tau, self.ts);
        self.d_slew = cfg.slew.map(|x| SlewRate::new(x, self.ts));
    }

    pub fn set_ts(&mut self, ts: f32) {
        self.ts = ts;
        self.d_lowpass.set_dt(ts);
        if let Some(slew) = &mut self.d_slew {
            slew.set_dt(ts);
        }
    }

    pub fn slew(mut self, slew: f32) -> Self {
        self.d_slew = Some(SlewRate::new(slew, self.ts));
        self
    }

    pub fn update(&mut self, reference: f32, measurement: f32, prediction: f32) -> f32 {

        // Calculate proportional error (with slight overdrive to account for
        // air resistance)

        let mut reference = reference * 1.02;

        let proportional = self.kp * (reference - measurement);

        // Take derivatives of measurement (negative because d(ref - meas)/dt)
        let measurement_lp = self.d_lowpass.update(measurement);
        let meas_derivative = - self.kd * (measurement_lp - self.prev_meas) / self.ts;
        self.prev_meas = measurement_lp;

        // Calculate error integral (if reference is small) This ensures we do
        // not carry over integration which occured for large references
        // (velocities) where air resistance and other strange dynamics likely
        // played a larger role than at a low reference.
        if prediction.abs() < 1.0 && self.integral_en {
            self.integral += self.ki * (prediction - measurement) * self.ts
                / (1.0 + self.lowpass.update(meas_derivative).abs());
        }

        // Optionally slew-rate limit the reference signal to prevent very quick
        // changes from being clipped by derivative controller.
        reference = self.d_slew.as_mut()
            .map_or(reference, |slew| slew.update(reference));

        // Calculate derivative
        let ref_derivative = self.kd * (reference - self.prev_ref) / self.ts;
        self.prev_ref = reference;

        // Sum all terms together. The meas_derivative_lp is subtracted because
        // the derivative terms is normally `d(ref
        // - meas)/dt` but in this case the derivative of ref and meas are done
        // separately, and therefore we need to use the negative sign here (for
        // clarity).
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
        self.ki = ki * kp;
        self.kd = kd * kp;
    }

    pub fn get_terms(&self) -> &PidTerms {
        &self.terms
    }
}
