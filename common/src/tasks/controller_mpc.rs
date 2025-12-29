use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::{SMatrix, SVector, SVectorView, SVectorViewMut, UnitQuaternion, Vector3, matrix, vector};
use tinympc_rs::{AntiSphere, Box, CircularCone, ProjectMulti, ProjectMultiExt as _, ProjectSingleExt as _, Solver, Sphere, policy::FixedPolicy};

use crate::{
    consts::GRAVITY,
    signals::ESKF_ESTIMATE,
    sync::{channel::Channel, watch::Watch},
};

#[allow(unused)]
#[cfg(not(feature = "arch-std"))]
use num_traits::Float as _;

pub const HX: usize = 100;
const HU: usize = HX - 5;
const NX: usize = 9;
const NU: usize = 3;
const DT: f32 = 0.1; // 10 Hz, 10 second horizon
const DD: f32 = 0.5 * DT * DT;

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
    pub const POS_Y: usize = 1;
    pub const POS_Z: usize = 2;
    pub const VEL_X: usize = 3;
    pub const VEL_Y: usize = 4;
    pub const VEL_Z: usize = 5;
    pub const ACC_X: usize = 6;
    pub const ACC_Y: usize = 7;
    pub const ACC_Z: usize = 8;
}

pub enum Message {
    /// The the reference position for some point in time and forward.
    SetPositionAt([f32; 3], Instant),
    SetInterseptPoint([f32; 3], Instant),
    ClearInterseptPoint,
}

pub static CHANNEL: Channel<Message, 2> = Channel::new();

mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct Parameters {
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
            rho: 8.0,
            cost_pos: Vec3 {
                x: 0.5, y: 0.5, z: 0.5,
            },
            cost_vel: Vec3 {
                x: 0.5, y: 0.5, z: 0.2,
            },
            cost_act: Vec3 {
                x: 0.5, y: 0.5, z: 0.5,
            },
            cost_dact: Vec3 {
                x: 1.0, y: 1.0, z: 1.0,
            }
        }
    );

    pub static TABLE: Table<Parameters> = Table::default("mpc");
}

pub const CONE_VERTEX: [f32; 3] = [100.0, 0.0, -50.0];
pub const CONE0_AXIS: [f32; 3] = [-2.0, 0.0, 1.0];
pub const CONE1_AXIS: [f32; 3] = [2.0, 0.0, -1.0];
pub const CONE_MU: f32 = 0.1;

struct InterceptPoint {
    point: Vector3<f32>,
    time: Instant,
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

#[embassy_executor::task]
pub async fn main() -> ! {
    let mut x_ref = SMatrix::<f32, NX, HX>::zeros();

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

    let r: SVector<f32, NU> = vector![params.cost_dact.x, params.cost_dact.y, params.cost_dact.z];

    let cache = FixedPolicy::new(
        params.rho,
        HX,
        &A,
        &B,
        &SMatrix::from_diagonal(&q),
        &SMatrix::from_diagonal(&r),
        &SMatrix::zeros(),
    )
    .unwrap();

    let mut mpc = Solver::new(A, B, cache).with_sys(system);

    mpc.config.relaxation = 1.0;
    mpc.config.max_iter = 1;

    let mut rcv_eskf_estimate = ESKF_ESTIMATE.receiver();

    // Positional entry cone for target interception
    let x_projector_cone0 = CircularCone::new()
        .vertex(CONE_VERTEX)
        .axis(CONE0_AXIS)
        .mu(CONE_MU)
        .dim_lift::<NX>([ax::POS_X, ax::POS_Y, ax::POS_Z]);

    // Positional exit cone for taget interception
    let x_projector_cone1 = CircularCone::new()
        .vertex(CONE_VERTEX)
        .axis(CONE1_AXIS)
        .mu(CONE_MU)
        .dim_lift::<NX>([ax::POS_X, ax::POS_Y, ax::POS_Z]);
    
    // Models the floor, prevents the drone from slamming into it
    let x_projector_floor = Box {
        lower: vector![f32::NEG_INFINITY],
        upper: vector![-1.0],
    }.dim_lift::<NX>([ax::POS_Z]);

    // Limit the velocity magnitude to 200 m/s
    let x_projector_speed = Sphere {
        center: vector![0.0, 0.0, 0.0],
        radius: 250.0,
    }.dim_lift::<NX>([ax::VEL_X, ax::VEL_Y, ax::VEL_Z]);

    // Each motor is able to produce 6.7 N. The vehicle weighs 630 grams.
    // So, four motors can accelerate with (6.73 [N] * 4) / 0.630 [g] = 42.5 [m/s²]
    // Limit the magnitude of total acceleration to 42 m/s²
    // Also have a keep-out constraint around gravity singularity.
    
    let x_projector_accel = (
        Sphere {
            center: vector![0.0, 0.0, GRAVITY],
            radius: MAX_ACCEL,
        },
        AntiSphere {
            center: vector![0.0, 0.0, GRAVITY],
            radius: 1.0
        }
    ).dim_lift::<NX>([ax::ACC_X, ax::ACC_Y, ax::ACC_Z]);
    
    // Combine projectors and extend throughout entire horizon
    let x_projector_bundle = (
        (x_projector_floor, x_projector_speed, &x_projector_accel).time_fixed(),
        x_projector_cone0.time_ranged(0..HX),
        x_projector_cone1.time_ranged(HX..HX)
    );

    let mut x_con: [tinympc_rs::Constraint<f32, _, NX, HX>; 1] = [x_projector_bundle.constraint_owned()];

    let u_projector_sphere = Sphere {
        center: vector![0.0, 0.0, 0.0],
        radius: GRAVITY * 0.5,
    };

    let u_projector_bundle = (u_projector_sphere,).time_fixed();
    let mut u_con: [tinympc_rs::Constraint<f32, _, NU, HU>; 1] = [u_projector_bundle.constraint_owned()];

    let gravity_vector = SVector::z() * GRAVITY;
    let mut control_sig = SVector::zeros();
    let mut x_now = SVector::zeros();

    Timer::after_millis(3500).await;

    info!("[mpc] Entering main loop");

    let mut intercept = Some(InterceptPoint {
        point: vector![0.0, 0.0, -100.0],
        time: Instant::MAX,
    });

    let mut ticker = Ticker::every(Duration::from_micros((DT * 1e6) as u64));
    loop {
        if let Either::First(message) = select(CHANNEL.receive(), ticker.next()).await {
            match message {
                Message::SetPositionAt(position, timestamp) => {
                    let mut position_ref = x_ref.fixed_view_mut::<3, HX>(0, 0);
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
                },
                Message::SetInterseptPoint(position, timestamp) => {
                    MPC_INTERCEPT_POS.send(Some(position));
                    intercept = Some(InterceptPoint {
                        point: position.into(),
                        time: timestamp,
                    });
                }
                Message::ClearInterseptPoint => {
                    info!("[controller_mpc] Clearing intercept point");
                    MPC_INTERCEPT_POS.send(None);
                    intercept = None;
                }
            }

            // Ensure channel is depleted before next tick
            continue;
        }

        let estimate = rcv_eskf_estimate.get().await;

        // Update and prepare intercept projectors and reference
        if let Some(intercept) = intercept.as_ref() {
            let projector = x_con[0].projector_mut();

            let split_idx = match intercept.time.checked_duration_since(Instant::now()) {
                Some(duration) => {
                    let num_secs = duration.as_micros() as f32 / 1e6;
                    ((num_secs / DT).round() as usize).min(HX - 1)
                }
                None => 0, // Timestamp in the past, update everything regardless
            };

            let axis = estimate.pos - intercept.point;

            // Correct the range of both cones (time)
            projector.1.range = 0..split_idx;
            projector.2.range = split_idx..HX;

            // Set the vertex position of both cones (where)
            projector.1.projector.projector.mut_vertex(intercept.point);
            projector.2.projector.projector.mut_vertex(intercept.point);

            // Set the axis of both cones (direction)
            projector.1.projector.projector.mut_axis(axis);
            projector.2.projector.projector.mut_axis(-axis);

            if split_idx != 0 {
                let ref_step_size = axis.unscale(-(split_idx as f32));
                info!("Step size: {:?}", ref_step_size);
                
                for (idx, mut ref_col) in x_ref.column_iter_mut().enumerate() {
                    let coord = estimate.pos + ref_step_size.scale(idx as f32);
                    ref_col.fixed_view_mut::<3, 1>(0,0).copy_from(&coord);
                }
            }
        } else {
            let projector = x_con[0].projector_mut();

            // Effectively disable cones using empty range
            projector.1.range = HX..HX;
            projector.2.range = HX..HX;
            shift_columns_left(&mut x_ref);
        }

        x_con[0].projector_mut().project_multi(&mut x_ref);

        x_now[0] = estimate.pos[0];
        x_now[1] = estimate.pos[1];
        x_now[2] = estimate.pos[2];
        x_now[3] = estimate.vel[0];
        x_now[4] = estimate.vel[1];
        x_now[5] = estimate.vel[2];
        x_now[6] = control_sig[0];
        x_now[7] = control_sig[1];
        x_now[8] = control_sig[2];

        // Run solver given constraints and reference
        let solution = mpc
            .initial_condition(x_now)
            .x_constraints(&mut x_con)
            .u_constraints(&mut u_con)
            .x_reference(&x_ref)
            .solve();

        // Integrate with the first delta_u
        control_sig += solution.u_prediction(0);

        // By adding gravity to the "ideal" mpc solution we get the global accel target
        // Also, we use a FUTURE actuation, to get ahead of the slower system dynamics!
        let global_accel_target = control_sig + solution.u_prediction(1) - gravity_vector;

        // This relies on the global_accel_target not having a magnitude near zero.
        let desired_thrust_dir = global_accel_target.normalize();

        // Determine the alignment between the desired attitude and the current one.
        // Use that to scale down the force target while ill-aligned
        let direction = estimate.att.transform_vector(&-SVector::z());
        let alignment_factor = direction.dot(&desired_thrust_dir).clamp(0.0, 1.0);

        let force_target = VEHICLE_MASS * global_accel_target.norm() * alignment_factor;

        let att_target = UnitQuaternion::rotation_between(&-SVector::z(), &desired_thrust_dir)
            .unwrap_or_else(UnitQuaternion::identity);

        let x_prediction = solution.x_prediction_full();
        let pos_pred = x_prediction.fixed_view::<3, HX>(ax::POS_X, 0);

        // Publish results
        MPC_TARGET_ATT.send(att_target);
        MPC_TARGET_ACC.send(global_accel_target.into());
        MPC_TARGET_FRC.send(force_target);
        MPC_REFERENCE.send(x_ref.clone());
        MPC_POS_PRED.send(pos_pred.clone_owned());
    }
}

pub static MPC_TARGET_ACC: Watch<[f32; 3]> = Watch::new();
pub static MPC_INTERCEPT_POS: Watch<Option<[f32; 3]>> = Watch::new();
pub static MPC_TARGET_ATT: Watch<UnitQuaternion<f32>> = Watch::new();
pub static MPC_TARGET_FRC: Watch<f32> = Watch::new();
pub static MPC_REFERENCE: Watch<SMatrix<f32, NX, HX>> = Watch::new();
pub static MPC_POS_PRED: Watch<SMatrix<f32, 3, HX>> = Watch::new();

/// This should be a parameterized configurable
const VEHICLE_MASS: f32 = 0.630;

const MAX_ACCEL: f32 = 40.0;