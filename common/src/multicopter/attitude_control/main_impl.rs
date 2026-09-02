use core::array::from_fn;
use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, with_timeout};
use nalgebra::{Matrix4, UnitQuaternion, Vector3, Vector4};

use super::command::AttitudeCommand;
use super::params::CtrlFlags;
use crate::airframe::DEV_QUAD_MOTOR_SETUP;
use crate::filters::angle_pid::Pid;
use crate::filters::rate_pid::RatePid;
use crate::filters::{Complementary, Lowpass, NthOrderLowpass, RampSmoother, SlewRate};
use crate::get_ctrl_freq;
use crate::signals::{self as sig, ThrottleCommand};
use crate::sync::channel::Channel;
use crate::sync::watch::{Receiver, Watch};
use crate::tasks::eskf::EskfEstimate;
use crate::types::measurements::Imu6DofData;
use crate::types::status::PidTerms;

use super::leaky_quaternion::LeakyQuaternion;
use super::params;

#[embassy_executor::task]
pub async fn parameter_update_notifier() -> ! {
    params::TABLE
        .run_notifier(|| CHANNEL.send(Message::UpdateParameters))
        .await
}

#[embassy_executor::task]
pub async fn integrator_enable_notifier() -> ! {
    let mut receiver = sig::ATTITUDE_INT_EN.receiver();
    loop {
        let enable = receiver.changed().await;
        CHANNEL.send(Message::EnableIntegrators(enable)).await;
    }
}

pub enum Message {
    UpdateParameters,
    EnableIntegrators(bool),
}

pub static CHANNEL: Channel<Message, 4> = Channel::new();

/// The control law selected by the last-seen [`AttitudeCommand`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum CtrlKind {
    Disabled,
    Rate,
    Angle,
}

fn ctrl_kind(cmd: &AttitudeCommand) -> CtrlKind {
    match cmd {
        AttitudeCommand::Disengage => CtrlKind::Disabled,
        AttitudeCommand::Rate { .. } => CtrlKind::Rate,
        AttitudeCommand::Angle { .. } => CtrlKind::Angle,
    }
}

#[derive(Debug)]
enum ModeState {
    Disabled,
    Rate {
        leaky_quat: LeakyQuaternion,
        ref_smoother: RampSmoother<Vector3<f32>>,
    },
    Angle {
        ref_smoother: RampSmoother<UnitQuaternion<f32>>,
    },
}

impl ModeState {
    const fn kind(&self) -> CtrlKind {
        match self {
            ModeState::Disabled => CtrlKind::Disabled,
            ModeState::Rate { .. } => CtrlKind::Rate,
            ModeState::Angle { .. } => CtrlKind::Angle,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TorqueSetpoint(pub [f32; 3]);

#[derive(Debug, Clone, Copy)]
pub struct RateSetpoint(pub [f32; 3]);

struct Controller<'a> {
    rcv_imu_data: Receiver<'a, Imu6DofData<f32>>,
    rcv_cmd: Receiver<'a, AttitudeCommand>,
    rcv_throttle: Receiver<'a, ThrottleCommand>,
    rcv_eksf_estimate: Receiver<'a, EskfEstimate>,
    rate_axes: [RateAxis; 3],
    angle_axes: [AngleAxis; 3],

    gyro_bias: Option<[f32; 3]>,
    flags: params::CtrlFlags,

    mode: ModeState,
    last_cmd: AttitudeCommand,
    last_cmd_time: Instant,
    cmd_timeout: Duration,
    thrust_lp: NthOrderLowpass<f32, 2>,
    leak_tc: f32,
    dt: f32,
    // TEMPORARY: Should be moved out of this task
    mixing_matrix: Matrix4<f32>,
    throttle: f32,
}

struct AngleAxis {
    pid: Pid<f32>,
}

struct RateAxis {
    qint_gain: f32,
    pid: RatePid,
    sp_slew: SlewRate<f32>,
    sp_lp: NthOrderLowpass<f32, 2>,
    pred_model: Lowpass<f32>,
    comp_filt: Complementary<f32>,
}

#[embassy_executor::task]
pub async fn main() -> ! {
    Controller::new().await.run().await
}

const MAX_GYR_MEAS: f32 = (0.95f32 * 2000.0).to_radians();
impl Controller<'_> {
    async fn new() -> Self {
        let spawner = SendSpawner::for_current_executor().await;
        let dt = 1.0 / get_ctrl_freq!() as f32;

        match parameter_update_notifier() {
            Ok(token) => spawner.spawn(token),
            _ => error!("[attitude_control] Failed to spawn parameter update notifier"),
        }

        match integrator_enable_notifier() {
            Ok(token) => spawner.spawn(token),
            _ => error!("[attitude_control] Failed to spawn integrator enable notifier"),
        }

        let params = params::TABLE.read().await;

        let mut controller = Controller {
            rcv_imu_data: sig::CAL_MULTI_IMU_DATA[0].receiver(),
            rcv_cmd: super::command::ATTITUDE_COMMAND.receiver(),
            rcv_throttle: sig::THROTTLE_COMMAND.receiver(),
            rcv_eksf_estimate: sig::ESKF_ESTIMATE.receiver(),
            rate_axes: [
                RateAxis {
                    qint_gain: params.x.qint,
                    pid: RatePid::new(params.x.kp, params.x.ki, params.x.kd, params.x.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.x.pred, dt),
                    comp_filt: Complementary::new(params.x.comp, dt),
                },
                RateAxis {
                    qint_gain: params.y.qint,
                    pid: RatePid::new(params.y.kp, params.y.ki, params.y.kd, params.y.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.y.pred, dt),
                    comp_filt: Complementary::new(params.y.comp, dt),
                },
                RateAxis {
                    qint_gain: params.z.qint,
                    pid: RatePid::new(params.z.kp, params.z.ki, params.z.kd, params.z.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.z.pred, dt),
                    comp_filt: Complementary::new(params.z.comp, dt),
                },
            ],
            angle_axes: [
                AngleAxis {
                    pid: Pid::new(15., 0., 0., true, dt),
                },
                AngleAxis {
                    pid: Pid::new(15., 0., 0., true, dt),
                },
                AngleAxis {
                    pid: Pid::new(25., 0., 0., true, dt),
                },
            ],
            gyro_bias: None,
            flags: params.flags.clone(),
            mode: ModeState::Disabled,
            last_cmd: AttitudeCommand::Disengage,
            last_cmd_time: Instant::now(),
            cmd_timeout: Duration::from_millis(params.cmd_timeout_ms as u64),
            thrust_lp: NthOrderLowpass::new(params.ref_lp, dt * 100.),
            leak_tc: params.att_leak_tc,
            dt: dt,

            mixing_matrix: DEV_QUAD_MOTOR_SETUP.into_mixing_matrix().unwrap(),
            throttle: 0.0,
        };

        // Disable all integral controllers initially; they are enabled when
        // the vehicle is detected as being in flight.
        controller
            .rate_axes
            .iter_mut()
            .for_each(|axis| axis.pid.enable_reset_integral(false));

        controller
    }

    async fn run(&mut self) -> ! {
        loop {
            match select(CHANNEL.receive(), self.rcv_imu_data.changed()).await {
                Either::First(message) => self.handle_message(message).await,
                Either::Second(imu_data) => {
                    if let Some(torque_setpoint) = self.on_imu_data(imu_data) {
                        TORQUE_SETPOINT.send(torque_setpoint);
                    }
                }
            }
        }
    }

    async fn handle_message(&mut self, message: Message) {
        match message {
            Message::UpdateParameters => {
                self.param_update().await;
            }
            Message::EnableIntegrators(enable) => {
                debug!("[mc/attitude_control]: Integrators enabled: {}", enable);
                for axis in self.rate_axes.iter_mut() {
                    axis.pid.enable_reset_integral(enable)
                }
            }
        }
    }

    async fn param_update(&mut self) {
        debug!("[mc/attitude_control]: Updating parameters");
        self.dt = 1.0 / get_ctrl_freq!() as f32;

        let Ok(params) = with_timeout(Duration::from_millis(10), params::TABLE.read()).await else {
            error!("[mc/attitude_control] Timed out waiting for lock on parameter table");
            return;
        };

        self.cmd_timeout = Duration::from_millis(params.cmd_timeout_ms as u64);
        self.set_parameters(&params);
    }

    /// Switch the internal control law based on a newly received command.
    ///
    /// Only switches when the *kind* of command changes, so that a
    /// continuous stream of e.g. `Rate` commands does not reset the
    /// reference-smoothing state.
    fn switch_cmd(&mut self, cmd: AttitudeCommand) {
        let new_kind = ctrl_kind(&cmd);
        if self.mode.kind() == new_kind {
            return;
        }

        debug!("[mc/attitude_control]: Setting control law: {:?}", new_kind);

        self.mode = match cmd {
            AttitudeCommand::Disengage => ModeState::Disabled,
            AttitudeCommand::Rate(target) => ModeState::Rate {
                leaky_quat: LeakyQuaternion::new(self.leak_tc, 1.0, self.dt),
                ref_smoother: RampSmoother::new(target),
            },
            AttitudeCommand::Angle(target) => ModeState::Angle {
                ref_smoother: RampSmoother::new(target),
            },
        };
    }

    fn set_parameters(&mut self, params: &params::Params) {
        // TODO: per-parameter diffing instead of resetting all filters
        for (param, axis) in [&params.x, &params.y, &params.z]
            .iter()
            .zip(&mut self.rate_axes)
        {
            axis.qint_gain = param.qint;
            axis.pid = RatePid::new(param.kp, param.ki, param.kd, param.dtau, self.dt);
            axis.pred_model = Lowpass::new(param.pred, self.dt);
            axis.comp_filt = Complementary::new(param.comp, self.dt);
            // These use parameters shared across all axes for now
            axis.sp_slew = SlewRate::new(params.ref_slew, self.dt);
            axis.sp_lp = NthOrderLowpass::new(params.ref_lp, self.dt);
        }

        self.leak_tc = params.att_leak_tc;
        self.flags = params.flags.clone();
    }

    fn on_imu_data(&mut self, mut imu_data: Imu6DofData<f32>) -> Option<TorqueSetpoint> {
        let now = Instant::now();

        // Consume any new attitude commands and throttle setpoints.
        if let Some(cmd) = self.rcv_cmd.try_changed() {
            self.last_cmd = cmd;
            self.last_cmd_time = now;
            self.switch_cmd(cmd);
        }

        if let Some(throttle) = self.rcv_throttle.try_changed() {
            self.throttle = throttle.0;
        }

        // Stale-command failsafe: if the active flight mode has not produced
        // a fresh command within `cmd_timeout`, disengage attitude control.
        // The flight-mode manager already supervises the mode itself; this is
        // a defense-in-depth guard on the data path.
        if now.saturating_duration_since(self.last_cmd_time) > self.cmd_timeout {
            self.mode = ModeState::Disabled;
        }

        // Pre rate-target filtering stage
        let raw_rate_setpoint = match &mut self.mode {
            ModeState::Disabled => {
                // Disengaged: no torque, and keep the motors at a safe idle.
                MOTORS_MIXED.send([0.0; 4]);
                return None;
            }
            ModeState::Rate { ref_smoother, .. } => {
                if let Some(AttitudeCommand::Rate(target_rate)) = self.rcv_cmd.try_changed() {
                    ref_smoother.add_sample(target_rate.into());
                }

                let rates = ref_smoother.get().into();
                RateSetpoint(rates)
            }
            ModeState::Angle { ref_smoother } => {
                if let Some(AttitudeCommand::Angle(target_angle)) = self.rcv_cmd.try_changed() {
                    ref_smoother.add_sample(target_angle.into());
                }

                let target = ref_smoother.get();
                self.angle_control(target)?
            }
        };

        // Remove bias form the gyro measurements if available
        if let Some(gyro_bias) = self.gyro_bias.as_ref() {
            for ax in 0..3 {
                imu_data.gyr[ax] -= gyro_bias[ax];
            }
        }

        // Filter rate target and compute predicted and estimated/fused angular rates
        let filt_rate_setpoint = self.filter_rate_target(raw_rate_setpoint);
        let (ff_rate_prediction, comp_rate_estimate) =
            self.rate_prediction(imu_data.gyr, raw_rate_setpoint);

        // Post rate-target filtering stage
        let rate_target = match &mut self.mode {
            ModeState::Rate {
                ref_smoother,
                leaky_quat,
            } if self
                .flags
                .intersects(CtrlFlags::LEAK_QUAT_FILT | CtrlFlags::LEAK_QUAT_PRED) =>
            {
                let leaky_rate_target = if self.flags.contains(CtrlFlags::LEAK_QUAT_FILT) {
                    filt_rate_setpoint.0
                } else {
                    ff_rate_prediction
                };

                let axis_error = leaky_quat.update(imu_data.gyr, leaky_rate_target, self.dt);

                RateSetpoint(from_fn(|ax| {
                    axis_error[ax] * self.rate_axes[ax].qint_gain + filt_rate_setpoint.0[ax]
                }))
            }
            _ => filt_rate_setpoint,
        };

        // Run inner-most angular rate PID loop, same for all modes
        let torque_target = TorqueSetpoint(core::array::from_fn(|axis| {
            let clamped_target_rate = rate_target.0[axis].clamp(-MAX_GYR_MEAS, MAX_GYR_MEAS);
            self.rate_axes[axis].pid.update(
                clamped_target_rate,
                comp_rate_estimate[axis],
                ff_rate_prediction[axis],
            )
        }));

        RATE_CONTROL_LOG.send(RateControlLog {
            timestamp_us: Instant::now().as_micros(),
            pid_terms: self.rate_axes.each_ref().map(|axis| *axis.pid.get_terms()),
            raw_setpoint: raw_rate_setpoint.0,
            filt_setpoint: filt_rate_setpoint.0,
            ff_prediction: ff_rate_prediction,
            comp_estimate: comp_rate_estimate,
        });

        let z_thrust_filt = self.thrust_lp.update(-self.throttle);
        let mixed_motors = self.mixing_matrix
            * Vector4::new(
                torque_target.0[0],
                torque_target.0[1],
                torque_target.0[2],
                z_thrust_filt,
            );

        MOTORS_MIXED.send(mixed_motors.into());

        Some(torque_target)
    }

    fn filter_rate_target(&mut self, mut rate_target: RateSetpoint) -> RateSetpoint {
        for ax in 0..3 {
            let axis = &mut self.rate_axes[ax];
            rate_target.0[ax] = axis.sp_slew.update(rate_target.0[ax]);
            rate_target.0[ax] = axis.sp_lp.update(rate_target.0[ax]);
        }
        rate_target
    }

    fn rate_prediction(
        &mut self,
        gyro_data: [f32; 3],
        rate_target: RateSetpoint,
    ) -> ([f32; 3], [f32; 3]) {
        let mut ff_pred_gyr = [0.0; 3];
        let mut comp_fuse_gyr = [0.0; 3];

        for ax in 0..3 {
            let axis = &mut self.rate_axes[ax];
            ff_pred_gyr[ax] = axis.pred_model.update(rate_target.0[ax]);
            comp_fuse_gyr[ax] = axis.comp_filt.update(gyro_data[ax], ff_pred_gyr[ax]);
        }

        (ff_pred_gyr, comp_fuse_gyr)
    }

    fn angle_control(&mut self, target: UnitQuaternion<f32>) -> Option<RateSetpoint> {
        let q_attitude = self.rcv_eksf_estimate.try_get()?.att;

        // Using the "quaternion error" rather than the euler angle error gives some
        // much nicer behavior where euler angles would normally experiece gimbal lock.
        let q_error = q_attitude.inverse() * target;
        let axis_error = q_error.scaled_axis();

        Some(RateSetpoint(from_fn(|ax| {
            self.angle_axes[ax].pid.update(axis_error[ax])
        })))
    }
}

pub static TORQUE_SETPOINT: Watch<TorqueSetpoint> = Watch::new();
pub static RATE_CONTROL_LOG: Watch<RateControlLog> = Watch::new();
pub static MOTORS_MIXED: Watch<[f32; 4]> = Watch::new();

#[derive(Debug, Clone)]
pub struct RateControlLog {
    pub timestamp_us: u64,
    pub pid_terms: [PidTerms; 3],
    pub raw_setpoint: [f32; 3],
    pub filt_setpoint: [f32; 3],
    pub ff_prediction: [f32; 3],
    pub comp_estimate: [f32; 3],
}
