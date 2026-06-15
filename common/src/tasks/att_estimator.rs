use core::sync::atomic::Ordering;

use ahrs::Ahrs;
use embassy_futures::select::{select, Either};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{get_ctrl_freq, signals as s, utils::rot_matrix::rot_x_180};

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "attitude_estimator";
    info!("{}: Task started", ID);

    // Task inputs
    let mut rcv_imu_data = s::CAL_MULTI_IMU_DATA[0].receiver();
    let mut rcv_mag_data = s::CAL_MULTI_MAG_DATA[0].receiver();

    // Task outputs
    let mut snd_ahrs_attitude_q = s::AHRS_ATTITUDE_Q.sender();
    let mut snd_ahrs_attitude = s::AHRS_ATTITUDE.sender();

    let dt = 1.0 / get_ctrl_freq!() as f32;

    // Madgwick gain (beta): weight given to accel/mag measurements vs gyro
    // integration. History:
    //   0.01 (default): too weak; D000036 showed 0.49 deg/s linear yaw drift.
    //   0.05: overcorrected; D000047 showed ~10 deg/s yaw-estimate drift on a
    //         physically stationary drone (post-BR-motor cleanup, gz ~ 0.1
    //         deg/s). Cause: mag readings shift under motor current because
    //         the D000115-120 cal was done with motors off, so mag sees
    //         current-induced field changes the cal can't correct.
    //   0.03 (BETA_INDOOR): compromise. Enough mag weight to arrest residual
    //         gyro bias integration over flight-test timescales, but not so
    //         much that motor-current-induced mag noise dominates the estimate.
    // BETA_OUTDOOR: stronger mag reliance for an absolute yaw reference,
    // selected at runtime only when the SC switch declares outdoor
    // (signals::MAG_TRUST_OUTDOOR). Safe to lean harder outside because the
    // building hard/soft-iron distortion that forced 0.03 indoors is absent;
    // the |B|norm gate below still rejects motor-EMI-corrupted samples.
    // Experimental starting point -- tune from logs.
    const BETA_INDOOR: f32 = 0.03;
    const BETA_OUTDOOR: f32 = 0.10;
    let mut ahrs = ahrs::Madgwick::new(dt, BETA_INDOOR);

    info!("{}: Entering main loop at {} Hz", ID, 1. / dt);
    '_infinite: loop {
        // Pick mag trust from the SC-gated outdoor flag (set by the device
        // ceiling-mode handler). beta_mut() updates the gain in place without
        // resetting the filter's attitude quaternion, so flipping SC mid-air
        // does not glitch the estimate.
        *ahrs.beta_mut() = if s::MAG_TRUST_OUTDOOR.load(Ordering::Relaxed) {
            BETA_OUTDOOR
        } else {
            BETA_INDOOR
        };

        // NOTE The madgwick filter implementation assumes a coordinate system
        // where the positive Z direction is up. This is opposite to the
        // coordinate system used in the drone firmware. To correct for this, we
        // rotate the IMU data by 180 degrees around the X axis.

        let attitude_q = match select(rcv_mag_data.changed(), rcv_imu_data.changed()).await {
            Either::First(mag_data) => {
                // Get the latest IMU data also
                let imu_data = rcv_imu_data.get().await;
                let gyr_data = rot_x_180(imu_data.gyr.into());
                let acc_data = rot_x_180(imu_data.acc.into());

                // |B|norm gate: compass_reader applies the ellipsoid cal so a
                // clean sample lands on the unit sphere (|B| ~ 1.0). Motor-
                // current EMI distorts the field direction, inflating or
                // shrinking the norm. Fuse only when the norm is within
                // +/- 10% of unit; otherwise fall back to IMU-only for this
                // cycle. Cheap to compute (no sqrt-of-sqrt) and safe --
                // skipping mag here reduces to the same gyro+accel path the
                // Madgwick filter already runs in the Second arm below.
                let mag_vec: Vector3<f32> = mag_data.into();
                let b_norm_sq = mag_vec.norm_squared();
                let use_mag = (0.81..=1.21).contains(&b_norm_sq);

                if use_mag {
                    let mag_rot = rot_x_180(mag_vec);
                    match ahrs.update(&gyr_data, &acc_data, &mag_rot) {
                        Err(_) => *ahrs.update_gyro(&gyr_data),
                        Ok(attitude_q) => *attitude_q,
                    }
                } else {
                    match ahrs.update_imu(&gyr_data, &acc_data) {
                        Err(_) => *ahrs.update_gyro(&gyr_data),
                        Ok(attitude_q) => *attitude_q,
                    }
                }
            }
            Either::Second(imu_data) => {
                let gyr_data = rot_x_180(imu_data.gyr.into());
                let acc_data = rot_x_180(imu_data.acc.into());
                match ahrs.update_imu(&gyr_data, &acc_data) {
                    Err(_) => *ahrs.update_gyro(&gyr_data),
                    Ok(attitude_q) => *attitude_q,
                }
            }
        };

        let attitude: [f32; 3] =
            (|(x, y, z)| rot_x_180(Vector3::new(x, y, z)))(attitude_q.euler_angles()).into();

        let attitude_q = UnitQuaternion::from_euler_angles(attitude[0], attitude[1], attitude[2]);

        // Mix signals and send to motor governer task
        snd_ahrs_attitude_q.send(attitude_q);
        snd_ahrs_attitude.send(attitude);
    }
}
