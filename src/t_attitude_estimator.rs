use ahrs::{Ahrs, Madgwick};
use nalgebra::Vector3;

use crate::messaging as msg;
use crate::common::rotation_matrices::rot_x_180 as rot;

#[embassy_executor::task]
pub async fn attitude_estimator() -> ! {

    // Input messages
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();
    let mut rcv_mag_data = msg::MAG_DATA.receiver().unwrap();

    // Output messages
    let snd_attitude_quat = msg::ATTITUDE_QUAT.sender();
    let snd_attitude_euler = msg::ATTITUDE_EULER.sender();

    // Period between IMU samples as seconds
    let imu_period = 1.0 / msg::CFG_LOOP_FREQUENCY.spin_get().await as f32;

    // Setup AHRS filter for attitude estimation
    let mut ahrs = Madgwick::new(imu_period, 0.002);

    '_infinite: loop {

        // Wait for new IMU data, then get MAG data if any
        // Here we assume values are calibrated and ready to use!
        let imu_reading = rcv_imu_data.changed().await;

        // Update AHRS filter (attitude estimation) and fuse MAG data if available and good
        let attitude_quat = match rcv_mag_data.try_changed() {
            Some(mag) if mag.magnitude() > 1.0 => {
                match ahrs.update(&rot(imu_reading.gyr),&rot(imu_reading.acc),&rot(mag),
                ) {
                    Ok(quat) => *quat,
                    _ => *ahrs.update_gyro(&rot(imu_reading.gyr)),
                }
            }
            _ => match ahrs.update_imu(&rot(imu_reading.gyr), &rot(imu_reading.acc)) {
                Ok(quat) => *quat,
                _ => *ahrs.update_gyro(&rot(imu_reading.gyr)),
            },
        };

        // Get attitude estimate in euler angles (and map from tuple to Vector3)
        let attitude_euler = (|(x, y, z)| rot(Vector3::new(x, y, z)))(attitude_quat.euler_angles());

        // Send attitude estimate to other tasks
        snd_attitude_quat.send(attitude_quat);
        snd_attitude_euler.send(attitude_euler);
    }
}