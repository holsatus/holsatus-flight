use core::sync::atomic::Ordering;
use embassy_time::Timer;

use crate::{consts::GRAVITY, filters::Lowpass, get_ctrl_freq};

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "if_estimator";
    info!("{}: Task started, entering main loop", STR_ID);

    let mut rcv_imu_data = crate::signals::CAL_MULTI_IMU_DATA[0].receiver();
    let mut rcv_motors_state = crate::signals::MOTORS_STATE.receiver();
    let mut rcv_motors_mixed = crate::tasks::controller_rate::RATE_MOTORS_MIXED.receiver();

    let dt = 1.0 / get_ctrl_freq!() as f32;

    let mut acc_z_filter = Lowpass::new(0.1, dt);

    loop {
        rcv_motors_state.get_and(|state| state.is_armed()).await;
        debug!("[if_estimator] Vehicle is armed, continuing");

        let force_target = rcv_motors_mixed.changed().await;
        let imu_data = rcv_imu_data.get().await;

        let accel = imu_data.acc[2].abs();
        let accel = acc_z_filter.update(accel);
        
        let force = force_target.iter().sum::<f32>();

        debug!("[if_estimator] Accel {}, force: {}", accel, force);

        let low_gravity = GRAVITY * 0.8;

        if force.abs() > VEHICLE_MASS * low_gravity && accel > low_gravity {
            debug!("[if_estimator] Vehicle detected as being IN FLIGHT!");
            crate::signals::IN_FLIGHT.store(true, Ordering::Relaxed);
            crate::signals::ATTITUDE_INT_EN.send(true);
        } else {
            Timer::after_millis(10).await;
            continue;
        }

        // Once the motors go disarmed, no longer consider us to be in flight
        rcv_motors_state.get_and(|state| state.is_disarmed()).await;
        crate::signals::IN_FLIGHT.store(false, Ordering::Relaxed);
        crate::signals::ATTITUDE_INT_EN.send(false);
        debug!("[if_estimator] Vehicle no longer in flight");
    }
}

const VEHICLE_MASS: f32 = 0.630;
