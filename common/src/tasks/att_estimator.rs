use ahrs::Ahrs;
use nalgebra::{UnitQuaternion, Vector3};

use crate::{signals as s, utils::rot_matrix::rot_x_180};

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "attitude_estimator";
    info!("{}: Task started", ID);

    // Task inputs
    // TODO - Use estimated attitude, not just IMU data
    let mut rcv_imu_data = s::CAL_MULTI_IMU_DATA[0].receiver().unwrap();
    let mut rcv_motors_state = s::MOTORS_STATE.receiver().unwrap();

    // Task outputs
    let snd_ahrs_attitude_q = s::AHRS_ATTITUDE_Q.sender();
    let snd_ahrs_attitude = s::AHRS_ATTITUDE.sender();

    let ts = crate::ANGLE_LOOP_DIV as f32 / crate::MAIN_LOOP_FREQ as f32;

    let mut ahrs = ahrs::Madgwick::new(ts, 0.01);

    info!("{}: Entering main loop at {} Hz", ID, 1. / ts);
    loop {
        let mut gyr_avg = Vector3::zeros();
        let mut acc_avg = Vector3::zeros();

        for _ in 0..crate::ANGLE_LOOP_DIV {
            let imu_data = rcv_imu_data.changed().await;
            gyr_avg += Into::<Vector3<_>>::into(imu_data.gyr);
            acc_avg += Into::<Vector3<_>>::into(imu_data.acc);
        }

        gyr_avg = gyr_avg.unscale(crate::ANGLE_LOOP_DIV as f32);
        acc_avg = acc_avg.unscale(crate::ANGLE_LOOP_DIV as f32);

        // let imu_data = rcv_imu_data.changed().await;
        // let gyr_avg = imu_data.gyr.into();
        // let acc_avg = imu_data.acc.into();
        // TODO
        // We should use this (along with prev attitude + gyro)
        // to compensate for linear accel produced by the drone
        let _motors_state = rcv_motors_state.get().await;

        let attitude_q = match ahrs.update_imu(&rot_x_180(gyr_avg), &rot_x_180(acc_avg)) {
            Err(_) => *ahrs.update_gyro(&rot_x_180(gyr_avg)),
            Ok(attitude_q) => *attitude_q,
        };

        let attitude: [f32; 3] =
            (|(x, y, z)| rot_x_180(Vector3::new(x, y, z)))(attitude_q.euler_angles()).into();

        let attitude_q = UnitQuaternion::from_euler_angles(attitude[0], attitude[1], attitude[2]);

        // Mix signals and send to motor governer task
        snd_ahrs_attitude_q.send(attitude_q);
        snd_ahrs_attitude.send(attitude);
    }
}
