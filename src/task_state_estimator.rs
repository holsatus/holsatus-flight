use ahrs::{Ahrs, Madgwick};
use defmt::*;
use nalgebra::{Quaternion, Unit, Vector3};

use crate::config;
use crate::channels;

static TASK_ID: &str = "[STATE_ESTIMATOR]";

#[embassy_executor::task]
pub async fn state_estimator(
    mut s_imu_reading: channels::ImuReadingSub,
    mut s_mag_reading: channels::MagReadingSub,
    p_attitude_sense: channels::AttitudeSensePub,
) {
    // TODO - Calculate expected acceleration (function of motor thrust dynamics and weight of drone)
    let exp_accel = Vector3::from([0., 0., 0.]);
    let mut prev_quaternion: Option<Unit<Quaternion<f32>>> = None;

    let mut ahrs = Madgwick::new(config::definitions::ATTITUDE_LOOP_TIME_SECS, 0.01, 0.001);

    info!("{}: Entering main loop", TASK_ID);
    loop {
        let mut imu_data = s_imu_reading.next_message_pure().await;

        // Use expected acceleration from motors to compensate in acceleration vector
        imu_data.acc = prev_quaternion.map_or_else(
            || imu_data.acc,
            |q| imu_data.acc - q.to_rotation_matrix() * exp_accel,
        );

        let ahrs_est = match s_mag_reading.try_next_message_pure() {
            
            // If magnetometer reading was available, use that
            Some(mag_reading) => {
                ahrs.update(
                    &negate::yz(imu_data.gyr),
                    &negate::yz(imu_data.acc),
                    &negate::yz(mag_reading),
                )
            },

            // Otherwise use IMU data only
            None => {
                ahrs.update_imu(
                    &negate::yz(imu_data.gyr),
                    &negate::yz(imu_data.acc),
                )
            },
        };

        // TODO - Investigate whether Madgwick filter can have y and z flipped by default
        #[cfg(not(feature = "mag"))]
        let ahrs_est = ahrs.update_imu(
            &negate::yz(imu_data.gyr), 
            &negate::yz(imu_data.acc
        ));

        match ahrs_est {
            Ok(q) => {
                let (roll, pitch, yaw) = q.euler_angles();
                let angle = negate::yz(Vector3::new(roll, pitch, yaw));
                p_attitude_sense.publish_immediate((angle, imu_data.gyr));

                prev_quaternion = Some(*q);
            }
            Err(e) => {
                warn!("{}: Error in AHRS filter -> {}", TASK_ID, e)
            }
        }
    }
}

/// Negate any combination of entries in a `Vector3<f32>`
#[allow(unused)]
mod negate {
    use nalgebra::Vector3;
    pub fn x(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(-vec.x, vec.y, vec.z) }
    pub fn y(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(vec.x, -vec.y, vec.z) }
    pub fn z(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(vec.x, vec.y, -vec.z) }
    pub fn xy(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(-vec.x, -vec.y, vec.z) }
    pub fn yz(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(vec.x, -vec.y, -vec.z) }
    pub fn xz(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(-vec.x, vec.y, -vec.z) }
    pub fn xyz(vec: Vector3<f32>) -> Vector3<f32> { Vector3::new(-vec.x, -vec.y, -vec.z) }
}

#[allow(unused)]
#[derive(Clone)]
enum EstimationError {
    MatrixSingular,
}
