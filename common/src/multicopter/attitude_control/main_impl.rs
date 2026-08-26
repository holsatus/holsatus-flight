use core::array::from_fn;
use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, with_timeout};
use nalgebra::UnitQuaternion;

use super::params::CtrlFlags;
use crate::filters::angle_pid::Pid;
use crate::filters::rate_pid::RatePid;
use crate::filters::{Complementary, FohSmoother, Lowpass, NthOrderLowpass, SlewRate};
use crate::get_ctrl_freq;
use crate::signals as sig;
use crate::sync::channel::Channel;
use crate::sync::watch::{Receiver, Watch};
use crate::tasks::eskf::EskfEstimate;
use crate::tasks::rc_binder::rates::Rates;
use crate::types::control::RcAnalog;
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
    SetRateSource(&'static Watch<[f32; 3]>),
    SetAngleSource(&'static Watch<UnitQuaternion<f32>>),
    SetModeKind(ModeKind),
    EnableIntegrators(bool),
}

pub static CHANNEL: Channel<Message, 4> = Channel::new();

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModeKind {
    Disabled,
    SticksRate,
    SticksAngle,
    DirectRate,
    DirectAngle,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum ModeState {
    Disabled,
    SticksRate(SticksRateState),
    SticksAngle(SticksAngleState),
    DirectRate,
    DirectAngle,
}

impl ModeState {
    const fn kind(&self) -> ModeKind {
        match self {
            ModeState::Disabled => ModeKind::Disabled,
            ModeState::SticksRate { .. } => ModeKind::SticksRate,
            ModeState::SticksAngle { .. } => ModeKind::SticksAngle,
            ModeState::DirectRate => ModeKind::DirectRate,
            ModeState::DirectAngle => ModeKind::DirectAngle,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct SticksAngleState {
    yaw_angle_rad: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct SticksRateState {
    leaky_quat: LeakyQuaternion,
}

#[derive(Debug, Clone, Copy)]
struct Sticks3D([f32; 3]);

#[derive(Debug, Clone, Copy)]
pub struct TorqueSetpoint(pub [f32; 3]);

#[derive(Debug, Clone, Copy)]
pub struct RateSetpoint(pub [f32; 3]);

#[derive(Debug, Clone, Copy)]
pub struct AngleSetpoint(pub UnitQuaternion<f32>);

struct RateController<'a> {
    rcv_imu_data: Receiver<'a, Imu6DofData<f32>>,
    rcv_rc_analog: Receiver<'a, RcAnalog>,
    rcv_rate_target: Receiver<'a, [f32; 3]>,
    rcv_angle_target: Receiver<'a, UnitQuaternion<f32>>,
    rcv_eksf_estimate: Receiver<'a, EskfEstimate>,
    rate_axes: [RateAxis; 3],
    angle_axes: [AngleAxis; 3],
    stick_foh: [FohSmoother; 3],

    gyro_bias: Option<[f32; 3]>,
    flags: params::CtrlFlags,

    mode: ModeState,
    leak_tc: f32,
    dt: f32,
}

struct AngleAxis {
    pid: Pid<f32>,
    rates: Rates,
}

struct RateAxis {
    qint_gain: f32,
    pid: RatePid,
    sp_slew: SlewRate<f32>,
    sp_lp: NthOrderLowpass<f32, 2>,
    pred_model: Lowpass<f32>,
    comp_filt: Complementary<f32>,
    rates: Rates,
}

#[embassy_executor::task]
pub async fn main() -> ! {
    RateController::start().await.run().await
}

const MAX_GYR_MEAS: f32 = (0.95f32 * 2000.0).to_radians();
impl RateController<'_> {
    async fn start() -> Self {
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

        RateController {
            rcv_imu_data: sig::CAL_IMU_DATA.receiver(),
            rcv_rc_analog: sig::RC_ANALOG_UNIT.receiver(),
            rcv_rate_target: sig::TRUE_RATE_SP.receiver(),
            rcv_angle_target: sig::TRUE_ATTITUDE_Q_SP.receiver(),
            rcv_eksf_estimate: sig::ESKF_ESTIMATE.receiver(),
            rate_axes: [
                RateAxis {
                    qint_gain: params.x.qint,
                    pid: RatePid::new(params.x.kp, params.x.ki, params.x.kd, params.x.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.x.pred, dt),
                    comp_filt: Complementary::new(params.x.comp, dt),
                    rates: params.x.rc.clone(),
                },
                RateAxis {
                    qint_gain: params.y.qint,
                    pid: RatePid::new(params.y.kp, params.y.ki, params.y.kd, params.y.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.y.pred, dt),
                    comp_filt: Complementary::new(params.y.comp, dt),
                    rates: params.y.rc.clone(),
                },
                RateAxis {
                    qint_gain: params.z.qint,
                    pid: RatePid::new(params.z.kp, params.z.ki, params.z.kd, params.z.dtau, dt),
                    sp_slew: SlewRate::new(params.ref_slew, dt),
                    sp_lp: NthOrderLowpass::new(params.ref_lp, dt),
                    pred_model: Lowpass::new(params.z.pred, dt),
                    comp_filt: Complementary::new(params.z.comp, dt),
                    rates: params.z.rc.clone(),
                },
            ],
            angle_axes: [
                AngleAxis {
                    pid: Pid::new(15., 0., 0., true, dt),
                    rates: Rates::Identity,
                },
                AngleAxis {
                    pid: Pid::new(15., 0., 0., true, dt),
                    rates: Rates::Identity,
                },
                AngleAxis {
                    pid: Pid::new(25., 0., 0., true, dt),
                    rates: Rates::Identity,
                },
            ],
            stick_foh: [
                FohSmoother::new(0.0),
                FohSmoother::new(0.0),
                FohSmoother::new(0.0),
            ],
            gyro_bias: None,
            flags: params.flags.clone(),
            mode: ModeState::Disabled,
            leak_tc: params.att_leak_tc,
            dt: dt,
        }
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

    /// Observe the value receivers to make sure the next call to (try_)changed is never stale.
    fn observe_receivers(&mut self) {
        match self.mode.kind() {
            ModeKind::Disabled => (),
            ModeKind::SticksAngle | ModeKind::SticksRate => _ = self.rcv_rc_analog.try_changed(),
            ModeKind::DirectAngle => _ = self.rcv_angle_target.try_changed(),
            ModeKind::DirectRate => _ = self.rcv_rate_target.try_changed(),
        }
    }

    async fn handle_message(&mut self, message: Message) {
        match message {
            Message::UpdateParameters => {
                self.param_update().await;
            }
            Message::SetRateSource(source) => {
                debug!("[mc/attitude_control]: Setting rate sp source");
                self.rcv_rate_target = source.receiver();
                self.observe_receivers();
            }
            Message::SetAngleSource(source) => {
                debug!("[mc/attitude_control]: Setting angle sp source");
                self.rcv_angle_target = source.receiver();
                self.observe_receivers();
            }
            Message::SetModeKind(mode_kind) => {
                self.switch_mode(mode_kind);
                self.observe_receivers();
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

        self.set_parameters(&params);
    }

    fn switch_mode(&mut self, mode_kind: ModeKind) {
        // Already in the desired state, just return
        if self.mode.kind() == mode_kind {
            return;
        }

        debug!("[mc/attitude_control]: Setting mode: {:?}", mode_kind);

        self.mode = match mode_kind {
            ModeKind::Disabled => ModeState::Disabled,
            ModeKind::SticksRate => ModeState::SticksRate(SticksRateState {
                leaky_quat: LeakyQuaternion::new(self.leak_tc, 1.0, self.dt),
            }),
            ModeKind::SticksAngle => {
                let attitude = self
                    .rcv_eksf_estimate
                    .try_get()
                    .map(|est| est.att)
                    .unwrap_or(UnitQuaternion::identity());

                ModeState::SticksAngle(SticksAngleState {
                    yaw_angle_rad: attitude.euler_angles().2,
                })
            }
            ModeKind::DirectRate => ModeState::DirectRate,
            ModeKind::DirectAngle => ModeState::DirectAngle,
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

        self.angle_axes[0].rates = params.x.rc;
        self.angle_axes[1].rates = params.y.rc;
        self.angle_axes[2].rates = params.z.rc;

        self.leak_tc = params.att_leak_tc;
        self.flags = params.flags.clone();
    }

    fn on_imu_data(&mut self, mut imu_data: Imu6DofData<f32>) -> Option<TorqueSetpoint> {
        let mut get_sticks = || {
            if let Some(analog) = self.rcv_rc_analog.try_changed() {
                let sticks = analog.roll_pitch_yaw();
                for ax in 0..3 {
                    self.stick_foh[ax].add_sample(sticks[ax]);
                }
            }
            Sticks3D(self.stick_foh.each_mut().map(|ax| ax.get()))
        };

        // Pre rate-target filtering stage
        let raw_rate_setpoint = match &mut self.mode {
            ModeState::Disabled => return None,
            ModeState::SticksRate(..) => {
                let sticks = get_sticks();

                RateSetpoint(from_fn(|ax| self.rate_axes[ax].rates.apply(sticks.0[ax])))
            }
            ModeState::SticksAngle(sticks_angle) => {
                let sticks = get_sticks();

                let [roll, pitch, yaw] =
                    from_fn(|axis| self.angle_axes[axis].rates.apply(sticks.0[axis]));

                // The yaw target is the integrated stick position
                sticks_angle.yaw_angle_rad += yaw * self.dt;
                let quaternion =
                    UnitQuaternion::from_euler_angles(roll, pitch, sticks_angle.yaw_angle_rad);
                let angle_setpoint = AngleSetpoint(quaternion);
                self.angle_control(angle_setpoint)?
            }
            ModeState::DirectRate => {
                let rates = self.rcv_rate_target.try_get()?;
                RateSetpoint(rates)
            }
            ModeState::DirectAngle => {
                let quaternion = self.rcv_angle_target.try_get()?;
                let angle_setpoint = AngleSetpoint(quaternion);
                self.angle_control(angle_setpoint)?
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
            ModeState::SticksRate(sticks_rate)
                if self
                    .flags
                    .intersects(CtrlFlags::LEAK_QUAT_FILT | CtrlFlags::LEAK_QUAT_PRED) =>
            {
                let leaky_rate_target = if self.flags.contains(CtrlFlags::LEAK_QUAT_FILT) {
                    filt_rate_setpoint.0
                } else {
                    ff_rate_prediction
                };

                let axis_error =
                    sticks_rate
                        .leaky_quat
                        .update(imu_data.gyr, leaky_rate_target, self.dt);

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

    fn angle_control(&mut self, target: AngleSetpoint) -> Option<RateSetpoint> {
        let q_attitude = self.rcv_eksf_estimate.try_get()?.att;

        // Using the "quaternion error" rather than the euler angle error gives some
        // much nicer behavior where euler angles would normally experiece gimbal lock.
        let q_error = q_attitude.inverse() * target.0;
        let axis_error = q_error.scaled_axis();

        Some(RateSetpoint(from_fn(|ax| {
            self.angle_axes[ax].pid.update(axis_error[ax])
        })))
    }
}

pub static TORQUE_SETPOINT: Watch<TorqueSetpoint> = Watch::new();
pub static RATE_CONTROL_LOG: Watch<RateControlLog> = Watch::new();

#[derive(Debug, Clone)]
pub struct RateControlLog {
    pub timestamp_us: u64,
    pub pid_terms: [PidTerms; 3],
    pub raw_setpoint: [f32; 3],
    pub filt_setpoint: [f32; 3],
    pub ff_prediction: [f32; 3],
    pub comp_estimate: [f32; 3],
}
