use embassy_futures::select::{Either, select};

use super::{Action, Controls, EnterError, FlightMode};
use crate::{
    multicopter::attitude_control::AttitudeCommand,
    signals::ThrottleCommand,
    sync::watch::{Receiver, Sender, Watch},
};

pub static ATTITUDE_SETPOINT: Watch<AttitudeCommand> = Watch::new();
pub static THROTTLE_SETPOINT: Watch<ThrottleCommand> = Watch::new();

/// Stabilized mode with direct attitude and thrust setpoint control
pub struct Stabilized {
    recv_attitude: Receiver<'static, AttitudeCommand>,
    recv_throttle: Receiver<'static, ThrottleCommand>,
    send_attitude: Sender<'static, AttitudeCommand>,
    send_throttle: Sender<'static, ThrottleCommand>,
}

impl FlightMode for Stabilized {
    async fn enter(controls: &Controls) -> Result<Self, EnterError> {
        Ok(Self {
            recv_attitude: ATTITUDE_SETPOINT.receiver(),
            recv_throttle: THROTTLE_SETPOINT.receiver(),
            send_attitude: controls.attitude,
            send_throttle: controls.throttle,
        })
    }

    async fn step(&mut self) -> Action {
        match select(self.recv_attitude.changed(), self.recv_throttle.changed()).await {
            Either::First(attitude) => self.send_attitude.send(attitude),
            Either::Second(throttle) => self.send_throttle.send(throttle),
        }

        Action::None
    }
}
