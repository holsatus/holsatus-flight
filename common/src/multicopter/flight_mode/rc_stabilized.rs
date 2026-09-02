use embassy_time::{Duration, Instant};
use nalgebra::UnitQuaternion;

use super::{Action, Controls, EnterError, FlightMode, Precondition};
use crate::{
    multicopter::attitude_control::AttitudeCommand,
    signals::{self as sig, ThrottleCommand},
    sync::watch::{Receiver, Sender},
    tasks::rc_binder::rates::Rates,
    types::control::RcAnalog,
    utils::func::wrap_rad,
};

pub mod params {
    use crate::tasks::{
        param_storage::Table,
        rc_binder::rates::{Actual, Linear, Rates},
    };

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        /// Rates applied to RC sticks in acro (rate) mode
        pub axis: [Rates; 3],
        /// Throttle mapping from unit RC throttle (0..1) to thrust setpoint
        pub thrt: Rates,
    }

    crate::const_default!(
        Params => {
            axis: [const { Rates::Actual(Actual::const_default()) }; 3],
            thrt: Rates::Linear(Linear { fact: 12.0, offs: 1.0 }),
        }
    );

    pub static TABLE: Table<Params> = Table::new("stab", Params::const_default());
}

/// Stabilized mode. RC sticks are mapped to an attitude (angle) setpoint.
pub struct RcStabilized {
    recv_rc_analog: Receiver<'static, RcAnalog>,
    send_attitude: Sender<'static, AttitudeCommand>,
    send_throttle: Sender<'static, ThrottleCommand>,
    yaw_angle_rad: f32,
    prev_step_time: Instant,
    axis_rates: [Rates; 3],
    throttle_rate: Rates,
}

impl FlightMode for RcStabilized {
    async fn enter(controls: &Controls) -> Result<Self, EnterError> {
        let mut missing = Precondition::empty();
        if sig::RC_ANALOG_UNIT.try_get().is_none() {
            missing.insert(Precondition::MANUAL_CONTROL);
        }
        if !missing.is_empty() {
            return Err(EnterError::Precondition(missing));
        }

        let params = params::TABLE.read().await.clone();

        let yaw_angle_rad = sig::ESKF_ESTIMATE
            .try_get()
            .map(|est| est.att.euler_angles().2)
            .unwrap_or_default();

        Ok(Self {
            recv_rc_analog: sig::RC_ANALOG_UNIT.receiver(),
            send_attitude: controls.attitude,
            send_throttle: controls.throttle,
            yaw_angle_rad,
            prev_step_time: Instant::now(),
            axis_rates: params.axis,
            throttle_rate: params.thrt,
        })
    }

    async fn step(&mut self) -> Action {
        let rc = self.recv_rc_analog.changed().await;

        // Compute delta time for yaw integration
        let now = Instant::now();
        let dt = now
            .saturating_duration_since(self.prev_step_time)
            .min(Duration::from_millis(100))
            .as_micros() as f32
            * 1e-6;

        // Apply RC "expo" mappings
        let mut rpy = rc.roll_pitch_yaw();
        for axis in 0..3 {
            rpy[axis] = self.axis_rates[axis].apply(rpy[axis]);
        }

        // Integrate stick deflection to yaw angle
        self.yaw_angle_rad = wrap_rad(self.yaw_angle_rad + rpy[2] * dt);

        let angle = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], self.yaw_angle_rad);
        let throttle = self.throttle_rate.apply(rc.throttle());

        critical_section::with(|_cs| {
            self.send_attitude.send(AttitudeCommand::Angle(angle));
            self.send_throttle.send(ThrottleCommand(throttle));
        });

        Action::None
    }
}
