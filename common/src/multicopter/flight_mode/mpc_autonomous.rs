//! Model-predictive position/attitude flight mode.
//!
//! This tracks a position reference over a fixed horizon, subject to velocity and
//! acceleration constraints, and produces a specific-force target. That target
//! is turned into an attitude setpoint (thrust direction) and a throttle
//! setpoint (thrust magnitude), which are published through [`Controls`].
//!
//! The horizon is advanced by a [`Ticker`], and a new position reference can be
//! pushed in at any time through [`CHANNEL`].

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{SMatrix, SVector, SVectorView, SVectorViewMut, UnitQuaternion, matrix, vector};
use tinympc_rs::{
    AntiSphere, Constraint, ProjectMulti, ProjectMultiExt as _, ProjectSingleExt as _, Solver,
    Sphere,
    policy::FixedPolicy,
    project::{dim::Lift, time::Fixed},
};

#[allow(unused)]
#[cfg(not(feature = "std"))]
use num_traits::Float as _;

use super::{Action, Controls, EnterError, FlightMode, Precondition};
use crate::{
    consts::GRAVITY,
    multicopter::attitude_control::AttitudeCommand,
    signals::{self as sig, ThrottleCommand},
    sync::{
        channel::{self, Channel},
        watch::{Receiver, Sender, Watch},
    },
    tasks::eskf::EskfEstimate,
};

pub const HX: usize = 50;
const HU: usize = HX - 5;
const NX: usize = 9;
const NU: usize = 3;
const DT: f32 = 0.1; // 10 Hz, 10 second horizon
const DD: f32 = 0.5 * DT * DT;

const MAX_ACCEL: f32 = 40.0;

const A: SMatrix<f32, NX, NX> = matrix![
    1., 0., 0., DT, 0., 0., DD, 0., 0.;
    0., 1., 0., 0., DT, 0., 0., DD, 0.;
    0., 0., 1., 0., 0., DT, 0., 0., DD;
    0., 0., 0., 1., 0., 0., DT, 0., 0.;
    0., 0., 0., 0., 1., 0., 0., DT, 0.;
    0., 0., 0., 0., 0., 1., 0., 0., DT;
    0., 0., 0., 0., 0., 0., 1., 0., 0.;
    0., 0., 0., 0., 0., 0., 0., 1., 0.;
    0., 0., 0., 0., 0., 0., 0., 0., 1.;
];

const B: SMatrix<f32, NX, NU> = matrix![
    DD, 0., 0.;
    0., DD, 0.;
    0., 0., DD;
    DT, 0., 0.;
    0., DT, 0.;
    0., 0., DT;
    1., 0., 0.;
    0., 1., 0.;
    0., 0., 1.;
];

fn system(mut xnext: SVectorViewMut<f32, NX>, x: SVectorView<f32, NX>, u: SVectorView<f32, NU>) {
    xnext.copy_from(&x); // Handles the identity-diagonal
    xnext[6] += u[0];
    xnext[7] += u[1];
    xnext[8] += u[2];
    xnext[0] += DT * x[3] + DD * xnext[6];
    xnext[1] += DT * x[4] + DD * xnext[7];
    xnext[2] += DT * x[5] + DD * xnext[8];
    xnext[3] += DT * xnext[6];
    xnext[4] += DT * xnext[7];
    xnext[5] += DT * xnext[8];
}

mod ax {
    pub const POS_X: usize = 0;
    pub const _POS_Y: usize = 1;
    pub const _POS_Z: usize = 2;
    pub const VEL_X: usize = 3;
    pub const VEL_Y: usize = 4;
    pub const VEL_Z: usize = 5;
    pub const ACC_X: usize = 6;
    pub const ACC_Y: usize = 7;
    pub const ACC_Z: usize = 8;
}

type XLiftSpeed = Lift<Sphere<f32, 3>, 3, NX>;
type XLiftAccel = Lift<(Sphere<f32, 3>, AntiSphere<f32, 3>), 3, NX>;
type XProjector = Fixed<(XLiftSpeed, XLiftAccel)>;
type UProjector = Fixed<(Sphere<f32, 3>,)>;

pub enum Message {
    /// The the reference position for some point in time and forward.
    SetPositionAt([f32; 3], Instant),
}

pub static CHANNEL: Channel<Message, 2> = Channel::new();

mod params {
    use crate::params::ParamTable;

    /// This should be a parameterized configurable
    const VEHICLE_MASS: f32 = 0.630;

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct Parameters {
        pub vehicle_mass: f32,
        pub rho: f32,
        pub cost_pos: Vec3,
        pub cost_vel: Vec3,
        pub cost_act: Vec3,
        pub cost_dact: Vec3,
    }

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct Vec3 {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    crate::const_default!(
        Parameters => {
            vehicle_mass: VEHICLE_MASS,
            rho: 2.0,
            cost_pos: Vec3 {
                x: 15.0, y: 15.0, z: 2.0,
            },
            cost_vel: Vec3 {
                x: 1.5, y: 1.5, z: 0.5,
            },
            cost_act: Vec3 {
                x: 0.0, y: 0.0, z: 0.0,
            },
            cost_dact: Vec3 {
                x: 0.5, y: 0.5, z: 0.5,
            }
        }
    );

    pub static TABLE: ParamTable<Parameters> = ParamTable::default("mpc");
}

/// Shifts all columns such that `column[i] <- column[i + 1]` with the last two being identical.
#[inline(always)]
pub(crate) fn shift_columns_left<const R: usize, const C: usize>(matrix: &mut SMatrix<f32, R, C>) {
    if C > 1 {
        let element_count = R * (C - 1);
        let ptr = matrix.as_mut_ptr();

        unsafe {
            core::ptr::copy(ptr.add(R), ptr, element_count);
        }
    }
}

/// Model-predictive position controller.
pub struct MpcAutonomous {
    recv_message: channel::Receiver<'static, Message, 2>,
    ticker: Ticker,
    recv_eskf_est: Receiver<'static, EskfEstimate>,
    send_attitude: Sender<'static, AttitudeCommand>,
    send_throttle: Sender<'static, ThrottleCommand>,

    vehicle_mass: f32,
    solver: Solver<f32, FixedPolicy<f32, NX, NU>, NX, NU, HX, HU>,
    x_ref: SMatrix<f32, NX, HX>,
    x_con: Constraint<f32, XProjector, NX, HX>,
    u_con: Constraint<f32, UProjector, NU, HU>,
    control_sig: SVector<f32, NU>,
    gravity_vector: SVector<f32, NU>,
}

impl FlightMode for MpcAutonomous {
    async fn enter(controls: &Controls) -> Result<Self, EnterError> {
        // The solver needs a state estimate to track against.
        if sig::ESKF_ESTIMATE.try_get().is_none() {
            return Err(EnterError::Precondition(Precondition::GLOBAL_POSITION));
        }

        let params = params::TABLE.read().await;

        let q: SVector<f32, NX> = vector![
            params.cost_pos.x,
            params.cost_pos.y,
            params.cost_pos.z,
            params.cost_vel.x,
            params.cost_vel.y,
            params.cost_vel.z,
            params.cost_act.x,
            params.cost_act.y,
            params.cost_act.z,
        ];

        let r: SVector<f32, NU> =
            vector![params.cost_dact.x, params.cost_dact.y, params.cost_dact.z];

        let cache = FixedPolicy::new(
            params.rho,
            HX,
            &A,
            &B,
            &SMatrix::from_diagonal(&q),
            &SMatrix::from_diagonal(&r),
            &SMatrix::zeros(),
        )
        .map_err(|_| {
            error!("[mc/mpc_autonomous] Failed to construct MPC policy");
            EnterError::Unimplemented
        })?;

        let mut solver = Solver::new(A, B, cache).with_sys(system);

        solver.config.relaxation = 1.0;
        solver.config.max_iter = 5;

        Ok(Self {
            recv_message: CHANNEL.receiver(),
            ticker: Ticker::every(Duration::from_micros((DT * 1e6) as u64)),
            recv_eskf_est: sig::ESKF_ESTIMATE.receiver(),
            send_attitude: controls.attitude,
            send_throttle: controls.throttle,
            vehicle_mass: params.vehicle_mass,
            solver,
            x_ref: SMatrix::zeros(),
            x_con: Self::construct_x_constraint(),
            u_con: Self::construct_u_constraint(),
            control_sig: SVector::zeros(),
            gravity_vector: SVector::z() * GRAVITY,
        })
    }

    async fn step(&mut self) -> Action {
        // A message only refreshes the reference; the solver runs on the next
        // tick, which guarantees the channel is drained before then.
        match select(self.recv_message.receive(), self.ticker.next()).await {
            Either::First(message) => self.on_message(message),
            Either::Second(_) => self.on_ticker().await,
        }

        Action::None
    }
}

impl MpcAutonomous {
    fn construct_x_constraint() -> Constraint<f32, XProjector, NX, HX> {
        // Limit the velocity magnitude to 200 m/s
        let x_projector_speed = Sphere {
            center: vector![0.0, 0.0, 0.0],
            radius: 10.0,
        }
        .dim_lift::<NX>([ax::VEL_X, ax::VEL_Y, ax::VEL_Z]);

        let x_projector_accel = (
            Sphere {
                center: vector![0.0, 0.0, GRAVITY],
                radius: MAX_ACCEL,
            },
            AntiSphere {
                center: vector![0.0, 0.0, GRAVITY],
                radius: 1.0,
            },
        )
            .dim_lift::<NX>([ax::ACC_X, ax::ACC_Y, ax::ACC_Z]);

        // Combine projectors and extend throughout entire horizon
        (x_projector_speed, x_projector_accel)
            .time_fixed()
            .constraint_owned()
    }

    fn construct_u_constraint() -> Constraint<f32, UProjector, NU, HU> {
        let u_projector_sphere = Sphere {
            center: vector![0.0, 0.0, 0.0],
            radius: GRAVITY,
        };

        (u_projector_sphere,).time_fixed().constraint_owned()
    }

    fn on_message(&mut self, message: Message) {
        match message {
            Message::SetPositionAt(position, timestamp) => {
                let mut position_ref = self.x_ref.fixed_view_mut::<3, HX>(0, 0);
                let from = match timestamp.checked_duration_since(Instant::now()) {
                    Some(duration) => {
                        let num_secs = duration.as_micros() as f32 / 1e6;
                        ((num_secs / DT).round() as usize).min(HX - 1)
                    }
                    None => 0, // Timestamp in the past, update everything regardless
                };
                for i in from..HX {
                    position_ref.set_column(i, &SVector::from(position));
                }
            }
        }
    }

    async fn on_ticker(&mut self) {
        shift_columns_left(&mut self.x_ref);

        let estimate = self.recv_eskf_est.get().await;

        self.x_con.projector_mut().project_multi(&mut self.x_ref);

        let x_now = nalgebra::vector![
            estimate.pos[0],
            estimate.pos[1],
            estimate.pos[2],
            estimate.vel[0],
            estimate.vel[1],
            estimate.vel[2],
            self.control_sig[0],
            self.control_sig[1],
            self.control_sig[2],
        ];

        // Run solver given constraints and reference
        let solution = self
            .solver
            .initial_condition(x_now)
            .x_constraints(core::slice::from_mut(&mut self.x_con))
            .u_constraints(core::slice::from_mut(&mut self.u_con))
            .x_reference(&self.x_ref)
            .solve();

        // Integrate with the first delta_u
        self.control_sig += solution.u_prediction(0);

        // By adding gravity to the "ideal" mpc solution we get the global accel target
        // Also, we use a FUTURE actuation, to get ahead of the slower system dynamics!
        let global_accel_target = self.control_sig + solution.u_prediction(1) - self.gravity_vector;

        // This relies on the global_accel_target not having a magnitude near zero.
        let desired_thrust_dir = global_accel_target.normalize();

        // Determine the alignment between the desired attitude and the current one.
        // Use that to scale down the force target while ill-aligned
        let direction = estimate.att.transform_vector(&-SVector::z());
        let alignment_factor = direction.dot(&desired_thrust_dir).clamp(0.0, 1.0);

        let force_target = self.vehicle_mass * global_accel_target.norm() * alignment_factor;

        let att_target = UnitQuaternion::rotation_between(&-SVector::z(), &desired_thrust_dir)
            .unwrap_or_else(UnitQuaternion::identity);

        let x_prediction = solution.x_prediction_full();
        let pos_pred = x_prediction.fixed_view::<3, HX>(ax::POS_X, 0);

        // Publish results
        MPC_TARGET_ATT.send(att_target);
        MPC_TARGET_ACC.send(global_accel_target.into());
        MPC_TARGET_FRC.send(force_target);
        MPC_REFERENCE.send(self.x_ref.clone());
        MPC_POS_PRED.send(pos_pred.clone_owned());

        info!("[mpc] force target: {}", force_target);

        critical_section::with(|_cs| {
            self.send_attitude.send(AttitudeCommand::Angle(att_target));
            self.send_throttle.send(ThrottleCommand(force_target));
        });
    }
}

pub static MPC_TARGET_ACC: Watch<[f32; 3]> = Watch::new();
pub static MPC_INTERCEPT_POS: Watch<Option<[f32; 3]>> = Watch::new();
pub static MPC_TARGET_ATT: Watch<UnitQuaternion<f32>> = Watch::new();
pub static MPC_TARGET_FRC: Watch<f32> = Watch::new();
pub static MPC_REFERENCE: Watch<SMatrix<f32, NX, HX>> = Watch::new();
pub static MPC_POS_PRED: Watch<SMatrix<f32, 3, HX>> = Watch::new();
