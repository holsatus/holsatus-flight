use core::ops::{AddAssign, SubAssign};
use num_traits::Float;

use super::Lowpass;

#[derive(Copy, Clone, Debug)]
pub struct Wrapping<F> {
    pub min: F,
    pub max: F,
}

/// The controller limits its output between `min` and `max`
#[derive(Copy, Clone, Debug)]
pub struct OutputLimit<F> {
    pub min: F,
    pub max: F,
}

/// The controller applies an anti windup method to the integrator
#[derive(Copy, Clone, Debug)]
pub enum AntiWindup<F> {
    Conditional((F, F)),
}

#[derive(Copy, Clone, Debug)]
pub struct PidConfig<F: Float> {
    pub kp: F,
    pub ki: F,
    pub kd: F,
    pub ideal: bool,
    pub wrapping: Option<(F, F)>,
    pub output_limit: Option<OutputLimit<F>>,
    pub anti_windup: Option<AntiWindup<F>>,
    pub lp_filter: Option<F>,
}

/// Pid controller implementation
#[derive(Copy, Clone)]
pub struct Pid<F: Float> {
    kp: F,
    ki: F,
    kd: F,
    ts: F,
    integral: F,
    integral_en: bool,
    prev_error: F,
    wrapping: Option<Wrapping<F>>,
    output_limit: Option<OutputLimit<F>>,
    anti_windup: Option<AntiWindup<F>>,
    lp_filter: Option<Lowpass<F>>,
}

impl<F: Float + SubAssign + AddAssign + PartialOrd> Pid<F> {
    /// ## PID controller
    ///
    /// Construct a new PID controller
    ///
    /// - `kp`: Proportional gain
    /// - `ki`: Integral gain
    /// - `kd`: Derivative gain
    /// - `ideal`: Controller on form `kp * ( 1 + ki/s + kd*s )`
    /// - `ts`: Sample time the controller is expected to run at
    #[must_use]
    pub fn new(kp: F, ki: F, kd: F, ideal: bool, ts: F) -> Self {
        Pid {
            kp,
            ki: if ideal { ki * kp } else { ki },
            kd: if ideal { kd * kp } else { kd },
            ts,
            integral: F::zero(),
            integral_en: true,
            prev_error: F::zero(),
            wrapping: None,
            output_limit: None,
            anti_windup: None,
            lp_filter: None,
        }
    }

    /// Construct a new PID controller from a configuration.
    #[must_use]
    pub fn with_config(config: PidConfig<F>, ts: F) -> Self {
        Pid {
            kp: config.kp,
            ki: if config.ideal {
                config.ki * config.kp
            } else {
                config.ki
            },
            kd: if config.ideal {
                config.kd * config.kp
            } else {
                config.kd
            },
            ts,
            integral: F::zero(),
            integral_en: true,
            prev_error: F::zero(),
            wrapping: config.wrapping.map(|w| Wrapping { min: w.0, max: w.1 }),
            output_limit: config.output_limit,
            anti_windup: config.anti_windup,
            lp_filter: config.lp_filter.map(|tau| Lowpass::new(tau, ts)),
        }
    }

    /// Set the wrapping limits for the controller input. If for example the input is an angle,
    /// the wrapping limits can be used to make the controller handles the transition from
    /// 360 degrees to 0 degrees gracefully.
    #[must_use]
    pub fn set_wrapping(mut self, min: F, max: F) -> Self {
        assert!(min < max, "The `min` value must be less than `max`");
        self.wrapping = Some(Wrapping { min, max });
        self
    }

    /// Set the output limits for the controller. The controller will not output a value less than
    /// `min` or greater than `max`.
    #[must_use]
    pub fn set_output_limit(mut self, min: F, max: F) -> Self {
        assert!(min < max, "The `min` value must be less than `max`");
        self.output_limit = Some(OutputLimit { min, max });
        self
    }

    /// Set the anti windup method for the controller. The anti windup method is used to prevent
    /// the integral from accumulating too much when the controller is saturated.
    #[must_use]
    pub fn set_anti_windup(mut self, anti_windup: AntiWindup<F>) -> Self {
        match anti_windup {
            AntiWindup::Conditional((min, max)) => {
                assert!(min < max, "The `min` value must be less than `max`")
            }
        }
        self.anti_windup = Some(anti_windup);
        self
    }

    /// Set a low-pass filter for the derivative controller. The derivative term is filtered with
    /// a time constant `tau` to reduce the impact of noise in the input.
    #[must_use]
    pub fn set_lp_filter(mut self, tau: F) -> Self {
        assert!(
            tau > F::zero(),
            "The time constant `tau` must be strictly positive"
        );
        self.lp_filter = Some(Lowpass::new(tau, self.ts));
        self
    }

    /// Change the gains of the individual controllers.
    #[must_use]
    pub fn set_gains(mut self, kp: F, ki: F, kd: F) -> Self {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self
    }

    /// Reset the integral to `0.0`.
    pub fn reset_integral(&mut self) {
        self.integral = F::zero();
    }

    /// Reset the integral to a specific value.
    pub fn reset_integral_to(&mut self, integral: F) {
        self.integral = integral;
    }

    /// Enables/disables the integral.
    pub fn enable_integral(&mut self, enable: bool) {
        self.integral_en = enable;
    }

    /// Enables/disables integrator, and resets integral to `0.0`.
    pub fn enable_reset_integral(&mut self, enable: bool) {
        self.enable_integral(enable);
        self.reset_integral();
    }

    /// Update the controller with an `error` value using a specified sampling time `ts`.
    pub fn update_ts(&mut self, mut error: F, ts: F) -> F {
        // Wrap/constrain input
        self.apply_wrapping(&mut error);

        // Proportional gain
        let proportional = self.kp * error;

        // Derivative gain
        let mut derivative = self.kd * (error - self.prev_error) / ts;

        // Save error for next iteration
        self.prev_error = error;

        // Low-pass filter
        self.apply_lp_filter(&mut derivative);

        // Integral gain and anti-windup
        self.apply_anti_windup(self.ki * error * ts, proportional + derivative);

        // Constrain output
        self.apply_output_limit(proportional + self.integral + derivative)
    }

    /// Update the controller with an `error` value.
    pub fn update(&mut self, error: F) -> F {
        self.update_ts(error, self.ts)
    }

    fn apply_wrapping(&mut self, error: &mut F) {
        if let Some(wrapping) = &self.wrapping {
            let minmaxdiff = wrapping.max - wrapping.min;

            while *error < wrapping.min {
                *error += minmaxdiff;
            }
            while *error > wrapping.max {
                *error -= minmaxdiff;
            }

            // Wrap previous error and integral
            if *error - self.prev_error < wrapping.min {
                self.prev_error -= minmaxdiff;
                self.integral -= minmaxdiff;
            } else if *error - self.prev_error > wrapping.max {
                self.prev_error += minmaxdiff;
                self.integral += minmaxdiff;
            }
        }
    }

    fn apply_output_limit(&mut self, input: F) -> F {
        if let Some(output_limit) = self.output_limit {
            if input < output_limit.min {
                output_limit.min
            } else if input > output_limit.max {
                output_limit.max
            } else {
                input
            }
        } else {
            input
        }
    }

    fn apply_anti_windup(&mut self, input: F, pd: F) {
        if self.integral_en {
            match self.anti_windup {
                Some(AntiWindup::Conditional((min, max))) => {
                    if pd > min && pd < max {
                        self.integral += input;
                    }
                }
                None => self.integral += input,
            }
        }
    }

    fn apply_lp_filter(&mut self, input: &mut F) {
        if let Some(lp_filter) = self.lp_filter.as_mut() {
            *input = lp_filter.update(*input)
        }
    }
}
