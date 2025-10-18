use core::array::from_fn;
use core::sync::atomic::Ordering;

use nalgebra::Vector4;

use crate::airframe::DEV_QUAD_MOTOR_SETUP;
use crate::filters::rate_pid::RatePid;
use crate::filters::{Complementary, Lowpass, NthOrderLowpass, SlewRate};
use crate::sync::watch::Watch;
use crate::types::status::PidTerms;
use crate::{get_ctrl_freq, get_or_warn, multi_receiver, signals as sig, NUM_IMU};

pub mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        /// Roll-axis related parameters
        pub x: AxisParameters,
        /// Pitch-axis related parameters
        pub y: AxisParameters,
        /// Yaw-axis related parameters
        pub z: AxisParameters,
        /// Slewrate limiter for reference signal
        pub ref_slew: f32,
        /// Low-pass filter for reference signal
        pub ref_lp: f32,
    }

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct AxisParameters {
        /// Proportional gain
        pub kp: f32,
        /// Integral gain
        pub ki: f32,
        /// Derivative gain
        pub kd: f32,
        /// Configuration flags
        pub cfg: AxisFlags,
        /// Time-constant of D-term LP filter
        pub dtau: f32,
        /// Prediction model time-constant
        pub pred: f32,
        /// Complementary filter time constant
        pub comp: f32,
    }

    impl Params {
        const fn const_default() -> Self {
            Params {
                x: AxisParameters::const_default(),
                y: AxisParameters::const_default(),
                z: AxisParameters::const_default(),
                ref_slew: 400.0,
                ref_lp: 0.001,
            }
        }
    }

    impl AxisParameters {
        const fn const_default() -> Self {
            AxisParameters {
                kp: 0.05,
                ki: 0.5,
                kd: 0.03,
                cfg: AxisFlags(0),
                dtau: 0.001,
                pred: 0.04,
                comp: 0.005,
            }
        }
    }

    #[derive(Clone, Debug, mav_param::Node)]
    pub struct AxisFlags(pub u8);

    bitflags::bitflags! {
        impl AxisFlags: u8 {
            const D_TERM_LP = 1 << 0;
            const REF_SLEW = 1 << 1;
            const COMP_PRED = 1 << 2;
        }
    }

    pub static TABLE: Table<Params> = Table::new("rate", Params::const_default());
}

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "rate_loop";
    info!("{}: Task started", ID);

    // Task inputs
    let mut rcv_imu_data = multi_receiver!(sig::CAL_MULTI_IMU_DATA, NUM_IMU);
    let mut rcv_z_thrust_sp = sig::TRUE_Z_THRUST_SP.receiver();
    let mut rcv_rate_sp = sig::TRUE_RATE_SP.receiver();
    let mut rcv_rate_int_en = sig::ATTITUDE_INT_EN.receiver();

    let mixer = DEV_QUAD_MOTOR_SETUP.into_mixing_matrix().unwrap();

    // Sampling time
    let dt: f32 = 1.0 / get_ctrl_freq!() as f32;

    // Load parameters
    let params = params::TABLE.read_initialized().await;

    // Define PID controllers
    let mut pid = [
        RatePid::new(params.x.kp, params.x.ki, params.x.kd, params.x.dtau, dt),
        RatePid::new(params.y.kp, params.y.ki, params.y.kd, params.y.dtau, dt),
        RatePid::new(params.z.kp, params.z.ki, params.z.kd, params.z.dtau, dt),
    ];

    // Slew-rate limiter for the reference signal
    let mut sp_slew_filt = [
        SlewRate::new(params.ref_slew, dt),
        SlewRate::new(params.ref_slew, dt),
        SlewRate::new(params.ref_slew, dt),
    ];

    // Lowpass limiter for the slew-limited reference signal
    let mut sp_lp_filt = [
        NthOrderLowpass::<_, 2>::new(params.ref_lp, dt),
        NthOrderLowpass::<_, 2>::new(params.ref_lp, dt),
        NthOrderLowpass::<_, 2>::new(params.ref_lp, dt),
    ];

    // Lowpass filters will act as feed-forward prediction.
    // We designed the closed loop system to have a bandwidth of 25Hz
    let mut pred_model = [
        Lowpass::new(params.x.pred, dt),
        Lowpass::new(params.y.pred, dt),
        Lowpass::new(params.z.pred, dt),
    ];

    // Complementary filter used to smoothen gyro data
    let mut comp = [
        Complementary::new(params.x.comp, dt),
        Complementary::new(params.y.comp, dt),
        Complementary::new(params.z.comp, dt),
    ];

    // No longer needed
    drop(params);

    // Disable all integral controllers initially
    pid.iter_mut()
        .for_each(|pid| pid.enable_reset_integral(false));

    // We await for the initial setpoints to be available
    let mut rate_sp = get_or_warn!(rcv_rate_sp).await;
    let mut z_thrust_sp = get_or_warn!(rcv_z_thrust_sp).await;
    let mut ref_filtered = rate_sp.clone();
    let mut comp_fuse_gyr = [0.0; 3];
    let mut ff_pred_gyr = [0.0; 3];

    info!("{}: Entering main loop", ID);
    loop {
        // Ensure we are reading from the correct IMU
        let active = sig::ACTIVE_IMU.load(Ordering::Relaxed);

        let imu_data = rcv_imu_data[active].changed().await;

        // Try to update rate and throttle setpoints
        rate_sp = rcv_rate_sp.try_get().unwrap_or(rate_sp);
        z_thrust_sp = rcv_z_thrust_sp.try_get().unwrap_or(z_thrust_sp);

        // Enable or disable integral controllers
        if let Some(enable) = rcv_rate_int_en.try_changed() {
            debug!("{}: Integrators enabled: {}", ID, enable);
            pid.iter_mut()
                .for_each(|pid| pid.enable_reset_integral(enable));
        }

        // Apply prediction, filtering and control pipeline
        let pid_torque: [f32; 3] = from_fn(|axis| {
            // Make prediction of the gyroscope based on (previous) reference
            ff_pred_gyr[axis] = pred_model[axis].update(ref_filtered[axis]);

            // Apply slew rate limiter to reference signal
            ref_filtered[axis] = sp_slew_filt[axis].update(rate_sp[axis]);

            // Slew-rate limit the reference signal (avoids D-term clipping)
            ref_filtered[axis] = sp_lp_filt[axis].update(ref_filtered[axis]);

            // Fuse gyro and prediction using complementary filter
            comp_fuse_gyr[axis] = comp[axis].update(imu_data.gyr[axis], ff_pred_gyr[axis]);

            // Update PID controller
            pid[axis].update(ref_filtered[axis], comp_fuse_gyr[axis], ff_pred_gyr[axis])
        });

        let vec = Vector4::new(pid_torque[0], pid_torque[1], pid_torque[2], -z_thrust_sp);

        let motors_mixed = (mixer * vec).into();

        // Extract PID terms for logging
        let pid_terms = from_fn(|axis| *pid[axis].get_terms());

        // Use critical section here to reduce number of interrupt enable/disable events
        critical_section::with(|_| {
            RATE_MOTORS_MIXED.send(motors_mixed);
            RATE_PID_TERMS.send(pid_terms);
            RATE_FF_PREDICT.send(ff_pred_gyr);
            RATE_REF_FILTERED.send(ref_filtered);
        });
    }
}

pub static RATE_MOTORS_MIXED: Watch<[f32; 4]> = Watch::new();
pub static RATE_PID_TERMS: Watch<[PidTerms; 3]> = Watch::new();
pub static RATE_FF_PREDICT: Watch<[f32; 3]> = Watch::new();
pub static RATE_REF_FILTERED: Watch<[f32; 3]> = Watch::new();
