use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};
use nalgebra::{UnitQuaternion, Vector3};

#[allow(unused_imports)]
use num_traits::Float as _;

use super::{Action, Controls, EnterError, FlightMode, Precondition};
use crate::{
    consts::GRAVITY,
    multicopter::attitude_control::AttitudeCommand,
    signals::{self as sig, ThrottleCommand},
    sync::watch::{Receiver, Sender, Watch},
};

/// Position setpoint [m] in the local NED frame.
///
/// Sending a new value wakes the position-hold loop so the setpoint is
/// applied immediately rather than waiting for the next control tick.
pub static POSITION_SP: Watch<[f32; 3]> = Watch::new();

/// Rudimentary position/velocity guidance + controller.
///
/// Cascade structure:
///   1. Guidance: position error → velocity setpoint (clamped).
///   2. Velocity controller: velocity error → acceleration setpoint.
///   3. The required specific force (acceleration − gravity) is turned into
///      an attitude command (thrust direction) and a throttle command
///      (thrust magnitude).
///
/// The vehicle attitude and thrust setpoints are published to the attitude
/// controller through the shared [`Controls`]. The control loop runs at a
/// steady tick, and immediately when a new setpoint arrives.
pub struct PositionHold {
    recv_setpoint: Receiver<'static, [f32; 3]>,
    send_attitude: Sender<'static, AttitudeCommand>,
    send_throttle: Sender<'static, ThrottleCommand>,
    setpoint: Vector3<f32>,
    yaw: f32,
    params: Params,
}

/// Control gains.
///
/// Hardcoded for now; promote these into a `mav_param::Tree` table (like
/// `stabilized::params`) once they are tuned on hardware.
#[derive(Debug, Clone, Copy)]
struct Params {
    /// Position error → velocity setpoint gain [1/s]
    kp_pos: f32,
    /// Velocity setpoint clamp [m/s]
    max_vel: f32,
    /// Velocity error → acceleration gain [1/s]
    kp_vel: f32,
    /// Damping on the measured velocity [1/s]
    kd_vel: f32,
    /// Maximum acceleration used to saturate the throttle setpoint [m/s²]
    max_accel: f32,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            kp_pos: 1.0,
            max_vel: 2.0,
            kp_vel: 2.5,
            kd_vel: 0.3,
            max_accel: 2.0 * GRAVITY,
        }
    }
}

/// Control loop period. The attitude inner loops run at the IMU rate; the
/// position/velocity cascade is intentionally slower.
const CONTROL_PERIOD: Duration = Duration::from_millis(10);

impl FlightMode for PositionHold {
    async fn enter(controls: &Controls) -> Result<Self, EnterError> {
        // A state estimate is required to define the initial hold point.
        let Some(est) = sig::ESKF_ESTIMATE.try_get() else {
            return Err(EnterError::Precondition(Precondition::GLOBAL_POSITION));
        };

        Ok(Self {
            recv_setpoint: POSITION_SP.receiver(),
            send_attitude: controls.attitude,
            send_throttle: controls.throttle,
            // Hold the current position until an external setpoint arrives.
            setpoint: est.pos,
            yaw: est.att.euler_angles().2,
            params: Params::default(),
        })
    }

    async fn step(&mut self) -> Action {
        // Run the controllers immediately when a new setpoint arrives, and
        // otherwise at a steady tick. If setpoints stream in faster than
        // CONTROL_PERIOD, the tick merely acts as an upper bound on the rate.
        match select(self.recv_setpoint.changed(), Timer::after(CONTROL_PERIOD)).await {
            Either::First(setpoint) => self.setpoint = Vector3::from(setpoint),
            Either::Second(()) => {}
        }

        self.run_control();

        Action::None
    }
}

impl PositionHold {
    fn run_control(&mut self) {
        let Some(est) = sig::ESKF_ESTIMATE.try_get() else {
            // No state estimate: disengage attitude control until one appears.
            self.send_attitude.send(AttitudeCommand::Disabled);
            self.send_throttle.send(ThrottleCommand(1.0));
            return;
        };

        // --- Guidance: position error → velocity setpoint ---
        let pos_err = self.setpoint - est.pos;
        let vel_sp = (pos_err * self.params.kp_pos)
            .map(|v| v.clamp(-self.params.max_vel, self.params.max_vel));

        // --- Velocity controller: velocity error → acceleration setpoint ---
        let vel_err = vel_sp - est.vel;
        let accel = vel_err * self.params.kp_vel - est.vel * self.params.kd_vel;

        // --- Required specific force in NED, and hence thrust direction ---
        // a = f + g  →  f_des = a_des − g, with g = [0, 0, GRAVITY] in NED.
        let f_des = accel - Vector3::new(0.0, 0.0, GRAVITY);

        // The thrust direction is along the body −z axis in NED.
        let (body_z, thrust_mag) = match f_des.try_normalize(1e-4) {
            Some(dir) => (-dir, f_des.norm()),
            None => (Vector3::new(0.0, 0.0, -1.0), 0.0),
        };

        // Thrust fraction → throttle, in the pseudo-scale used by the RC
        // modes (1.0 idle … 13.0 full).
        let thrust_frac = (thrust_mag / self.params.max_accel).clamp(0.0, 1.0);
        let throttle = 1.0 + thrust_frac * 12.0;

        let attitude = attitude_from_body_z(body_z, self.yaw);

        critical_section::with(|_cs| {
            self.send_attitude.send(AttitudeCommand::Angle(attitude));
            self.send_throttle.send(ThrottleCommand(throttle));
        });
    }
}

/// Build the attitude whose body-z axis points along `body_z` (in NED) and
/// whose heading (the body-x axis projected onto the plane perpendicular to
/// `body_z`) points at `yaw`.
fn attitude_from_body_z(body_z: Vector3<f32>, yaw: f32) -> UnitQuaternion<f32> {
    let z = body_z.normalize();
    let yaw_vec = Vector3::new(yaw.cos(), yaw.sin(), 0.0);

    // Project the desired heading onto the plane perpendicular to body-z.
    let x_unnorm = yaw_vec - z * yaw_vec.dot(&z);
    let x = match x_unnorm.try_normalize(1e-6) {
        Some(x) => x,
        // Degenerate: body-z is (near-)parallel to the heading, i.e. the
        // vehicle is tipped ~90°. Any perpendicular direction will do.
        None => z.cross(&Vector3::z()).normalize(),
    };
    let y = z.cross(&x);

    UnitQuaternion::from_basis_unchecked(&[x, y, z])
}
