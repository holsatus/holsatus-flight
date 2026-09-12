use embassy_time::{Duration, Instant};

use super::{Action, Controls, EnterError, FlightMode, Precondition};
use crate::{
    multicopter::attitude_control::AttitudeCommand,
    signals::{RC_ANALOG_UNIT, ThrottleCommand},
    sync::watch::{Receiver, Sender},
    tasks::rc_binder::rates::Rates,
    types::control::RcAnalog,
};

pub mod params {
    use crate::{
        params::ParamTable,
        tasks::rc_binder::rates::{Actual, Linear, Rates},
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

    pub static TABLE: ParamTable<Params> = ParamTable::default("acro");
}

/// Acro mode. RC sticks are mapped to angular-rate setpoints.
pub struct RcAcrobatic {
    recv_rc_analog: Receiver<'static, RcAnalog>,
    send_attitude: Sender<'static, AttitudeCommand>,
    send_throttle: Sender<'static, ThrottleCommand>,
    axis_rates: [Rates; 3],
    throttle_rate: Rates,
}

// Pull this into a more globally available place. And make it more complete.
fn test_precondition(cond: Precondition) -> Result<(), Precondition> {
    let mut failed = Precondition::empty();

    if cond.contains(Precondition::MANUAL_CONTROL) {
        if RC_ANALOG_UNIT.try_get().is_none_or(|rc| {
            Instant::from_micros(rc.timestamp_us).elapsed() > Duration::from_millis(100)
        }) {
            failed.insert(Precondition::MANUAL_CONTROL);
        }
    }

    if failed.is_empty() {
        Ok(())
    } else {
        Err(failed)
    }
}

impl FlightMode for RcAcrobatic {
    async fn enter(controls: &Controls) -> Result<Self, EnterError> {
        test_precondition(Precondition::MANUAL_CONTROL | Precondition::GYR_CALIBRATED)
            .map_err(EnterError::Precondition)?;

        let params = params::TABLE.read().await;

        Ok(Self {
            recv_rc_analog: RC_ANALOG_UNIT.receiver(),
            send_attitude: controls.attitude,
            send_throttle: controls.throttle,
            axis_rates: params.axis,
            throttle_rate: params.thrt,
        })
    }

    async fn step(&mut self) -> Action {
        let rc = self.recv_rc_analog.changed().await;

        // Apply RC "expo" mappings
        let mut rate = rc.roll_pitch_yaw();
        for axis in 0..3 {
            rate[axis] = self.axis_rates[axis].apply(rate[axis]);
        }

        let throttle = self.throttle_rate.apply(rc.throttle());

        critical_section::with(|_cs| {
            self.send_attitude.send(AttitudeCommand::Rate(rate.into()));
            self.send_throttle.send(ThrottleCommand(throttle));
        });

        Action::None
    }

    const PARAMS: Option<&'static crate::params::ParamTable<dyn mav_param::Node>> =
        Some(&params::TABLE);
}
