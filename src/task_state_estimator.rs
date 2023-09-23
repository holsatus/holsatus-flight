use ahrs::{Madgwick,Ahrs};
use defmt::*;
use nalgebra::{Quaternion, Unit, Vector3};

use crate::channels;
use crate::cfg;

static TASK_ID : &str = "STATE_ESTIMATOR";

#[embassy_executor::task]
pub async fn state_estimator(
    mut s_imu_reading: channels::ImuReadingSub,
    p_attitude_sense: channels::AttitudeSensePub,
) {

    let exp_accel = Vector3::from([0.,0.,0.]);

    let mut ahrs = Madgwick::new(cfg::ATTITUDE_LOOP_TIME_SECS, 0.01, 0.001);

    let mut prev_quaternion : Option<Unit<Quaternion<f32>>> = None;

    info!("{} : Entering main loop",TASK_ID);
    loop {
        let mut imu_data = s_imu_reading.next_message_pure().await;

        // TODO Investigate how the ahrs library can accept NED coordinates
        imu_data.acc.y *= -1.0;
        imu_data.acc.z *= -1.0;

        imu_data.gyr.y *= -1.0;
        imu_data.gyr.z *= -1.0;

        let acceleration = prev_quaternion.map_or_else(
            ||imu_data.acc,
            |q|{imu_data.acc - q.to_rotation_matrix()*exp_accel}
        );

        #[cfg(feature = "mag")]
        let ahrs_est = ahrs.update(&imu_data.gyr, &imu_data.acc, &imu_data.mag);

        #[cfg(not(feature = "mag"))]
        let ahrs_est = ahrs.update_imu(&imu_data.gyr, &acceleration);

        match ahrs_est {
            Ok(q) => {
                let (roll,pitch,yaw) = q.euler_angles();
                let angle = Vector3::new( roll,-pitch,-yaw );
                p_attitude_sense.publish_immediate((angle,imu_data.gyr));

                prev_quaternion = Some(*q);
            },
            Err(e) => {
                warn!("{} : Error in AHRS filter -> {}",TASK_ID,e)
            },
        }


    }
}

#[allow(unused)]
#[derive(Clone)]
enum EstimationError {
    MatrixSingular,
}