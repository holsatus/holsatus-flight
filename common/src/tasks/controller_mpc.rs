use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker, Timer};
use nalgebra::{matrix, vector, Quaternion, SMatrix, SVector, Unit, UnitQuaternion};
use tinympc_rs::{cache::SingleCache, AntiSphere, Box, Project, ProjectExt as _, Sphere, TinyMpc};

use crate::{
    consts::GRAVITY,
    signals::ESKF_ESTIMATE,
    sync::{channel::Channel, watch::Watch},
};

#[allow(unused)]
#[cfg(not(feature = "arch-std"))]
use num_traits::Float as _;

pub const HX: usize = 20;
const HU: usize = HX - 10;
const NX: usize = 9;
const NU: usize = 3;
const DT: f32 = 0.05; // 20 Hz, 5 second horizon
const DD: f32 = 0.5 * DT * DT;

// The approximate time constant for changing attitude
const TAU: f32 = 0.1;
const LP: f32 = TAU / (DT + TAU);

const A: SMatrix<f32, NX, NX> = matrix![
    1., 0., 0., DT, 0., 0., DD, 0., 0.;
    0., 1., 0., 0., DT, 0., 0., DD, 0.;
    0., 0., 1., 0., 0., DT, 0., 0., DD;
    0., 0., 0., 1., 0., 0., DT, 0., 0.;
    0., 0., 0., 0., 1., 0., 0., DT, 0.;
    0., 0., 0., 0., 0., 1., 0., 0., DT;
    0., 0., 0., 0., 0., 0., LP, 0., 0.;
    0., 0., 0., 0., 0., 0., 0., LP, 0.;
    0., 0., 0., 0., 0., 0., 0., 0., LP;
];

const B: SMatrix<f32, NX, NU> = matrix![
    0., 0., 0.;
    0., 0., 0.;
    0., 0., 0.;
    0., 0., 0.;
    0., 0., 0.;
    0., 0., 0.;
    (1. - LP), 0., 0.;
    0., (1. - LP), 0.;
    0., 0., (1. - LP);
];

const Q: SVector<f32, NX> = vector![9., 9., 9., 2., 2., 2., 1., 1., 1.];
const R: SVector<f32, NU> = vector![5., 5., 5.];
const RHO: f32 = 16.0;

type Cache = SingleCache<f32, NX, NU>;
type Mpc = TinyMpc<f32, Cache, NX, NU, HX, HU>;

pub enum Message {
    /// The the reference position for some point in time and forward.
    SetPositionAt([f32; 3], Instant),
}

pub static CHANNEL: Channel<Message, 10> = Channel::new();

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
    let mut reference = SMatrix::<f32, NX, HX>::zeros();
    let mut ticker = Ticker::every(Duration::from_micros((DT * 1e6) as u64));

    let cache = Cache::new(
        RHO,
        HX,
        &A,
        &B,
        &SMatrix::from_diagonal(&Q),
        &SMatrix::from_diagonal(&R),
        &SMatrix::zeros(),
    )
    .unwrap();

    let mut mpc = Mpc::new(A, B, cache);

    mpc.config.relaxation = 1.8;
    mpc.config.max_iter = 5;

    let mut rcv_eskf_estimate = ESKF_ESTIMATE.receiver();
    let mut rcv_imu_data = crate::signals::CAL_MULTI_IMU_DATA[0].receiver();

    // Keep-out zone
    let x_projector_asphere = AntiSphere {
        center: vector![
            Some(-50.0),
            Some(0.0),
            Some(-50.0),
            None,
            None,
            None,
            None,
            None,
            None
        ],
        radius: 20.0,
    };

    // Absolute velocity limit of 50 m/s
    let x_projector_sphere = Sphere {
        center: vector![
            None,
            None,
            None,
            Some(0.0),
            Some(0.0),
            Some(0.0),
            None,
            None,
            None
        ],
        radius: 50.0,
    };

    // Models the floor, prevents the drone from slamming into it
    let x_projector_floor = Box {
        lower: vector![None, None, None, None, None, None, None, None, None],
        upper: vector![None, None, Some(0.0), None, None, None, None, None, None],
    };

    let x_projector_bundle = (x_projector_sphere, x_projector_floor, x_projector_asphere);
    let mut x_con = [x_projector_bundle.constraint()];

    // Limit the amount of acceleration in any direction. Bias downwards since gravity is helping in that case.
    let u_projector_sphere = Sphere {
        center: vector![Some(0.0), Some(0.0), Some(0.0)],
        radius: GRAVITY * 8.0, // Allow 4g of accel
    };

    // Should prevent the MPC from causing the vehicle to spin too much
    let u_projector_asphere = AntiSphere {
        center: vector![Some(0.0), Some(0.0), Some(GRAVITY)],
        radius: GRAVITY * 0.5,
    };

    let u_projector_bundle = (u_projector_sphere, u_projector_asphere);
    let mut u_con = [u_projector_bundle.constraint()];

    let gravity_vector = SVector::z() * GRAVITY;

    Timer::after_millis(2500).await;

    info!("[mpc] Entering main loop");

    loop {
        if let Either::First(message) = select(CHANNEL.receive(), ticker.next()).await {
            match message {
                Message::SetPositionAt(position, timestamp) => {
                    let mut position_ref = reference.fixed_view_mut::<3, HX>(0, 0);
                    let from = match timestamp.checked_duration_since(Instant::now()) {
                        Some(duration) => {
                            let num_secs = duration.as_micros() as f32 / 1e6;
                            ((num_secs / DT) as usize).min(HX - 1)
                        }
                        None => 0, // Timestamp in the past, update everything regardless
                    };
                    for i in from..HX {
                        position_ref.set_column(i, &SVector::from(position));
                    }
                }
            }
            continue;
        }

        shift_columns_left(&mut reference);
        x_projector_bundle.project(&mut reference);
        MPC_REFERENCE.send(reference.clone());

        // Calculate this conversion
        let estimate = rcv_eskf_estimate.get().await;
        let attitude = Quaternion::from_vector(estimate.att.into());
        let attitude = Unit::from_quaternion(attitude);
        let local_accel = SVector::from(rcv_imu_data.get().await.acc) + gravity_vector;
        let world_accel = attitude.inverse_transform_vector(&local_accel);

        let mut x_now = SVector::zeros();

        x_now[0] = estimate.pos[0];
        x_now[1] = estimate.pos[1];
        x_now[2] = estimate.pos[2];
        x_now[3] = estimate.vel[0];
        x_now[4] = estimate.vel[1];
        x_now[5] = estimate.vel[2];
        x_now[6] = world_accel[0];
        x_now[7] = world_accel[1];
        x_now[8] = world_accel[2];

        // Run solver
        let time = Instant::now();
        let solution = mpc
            .initial_condition(x_now)
            .x_reference(&reference)
            .x_constraints(&mut x_con)
            .u_constraints(&mut u_con)
            .solve();
        let _dur = time.elapsed();

        info!(
            "[controller_mpc] Elapsed time: {}, iters: {}",
            _dur.as_micros() as f32 / 1e6,
            solution.iterations,
        );

        // By adding gravity to the "ideal" mpc solution we get the global accel target
        let global_accel_target = solution.u_now() - gravity_vector;

        let att_target =
            UnitQuaternion::rotation_between(&-SVector::z(), &global_accel_target.normalize())
                .unwrap_or_else(UnitQuaternion::identity);

        // Set desired_force as thrust target (f = m * a)
        let force_target = VEHICLE_MASS * global_accel_target.norm();

        let x_prediction = solution.x_prediction();
        let pos_pred = x_prediction.fixed_view::<3, HX>(0, 0);

        // Publish results
        MPC_TARGET_ATT.send(att_target);
        MPC_TARGET_ACC.send(global_accel_target.into());
        MPC_TARGET_FRC.send(force_target);
        MPC_POS_PRED.send(pos_pred.clone_owned());
    }
}

pub static MPC_TARGET_ACC: Watch<[f32; 3]> = Watch::new();
pub static MPC_TARGET_ATT: Watch<UnitQuaternion<f32>> = Watch::new();
pub static MPC_TARGET_FRC: Watch<f32> = Watch::new();
pub static MPC_REFERENCE: Watch<SMatrix<f32, NX, HX>> = Watch::new();
pub static MPC_POS_PRED: Watch<SMatrix<f32, 3, HX>> = Watch::new();

/// This should be a parameterized configurable
const VEHICLE_MASS: f32 = 0.566;
