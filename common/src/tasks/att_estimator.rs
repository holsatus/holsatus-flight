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

    let mut ahrs = ahrs::Madgwick::new(dt, 0.01);
    let q_rot_x_180 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f32::consts::PI);

    info!("{}: Entering main loop at {} Hz", ID, 1. / dt);
    '_infinite: loop {
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
                let mag_data = rot_x_180(mag_data.into());
                match ahrs.update(&gyr_data, &acc_data, &mag_data) {
                    Err(_) => *ahrs.update_gyro(&gyr_data),
                    Ok(attitude_q) => *attitude_q,
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

        // Convert the AHRS output back to firmware frame with quaternion composition.
        // Rotating Euler angles as a 3D vector is not a valid frame transform.
        let attitude_q = q_rot_x_180 * attitude_q;
        let attitude: [f32; 3] = attitude_q.euler_angles().into();

        // Mix signals and send to motor governer task
        snd_ahrs_attitude_q.send(attitude_q);
        snd_ahrs_attitude.send(attitude);
    }
}
