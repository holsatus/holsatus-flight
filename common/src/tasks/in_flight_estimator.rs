use core::sync::atomic::Ordering;
use embassy_time::Timer;

use crate::consts::GRAVITY;

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "if_estimator";
    info!("{}: Task started, entering main loop", STR_ID);

    let mut rcv_imu_data = crate::signals::CAL_MULTI_IMU_DATA[0].receiver();
    let mut rcv_motors_state = crate::signals::MOTORS_STATE.receiver();
    let mut rcv_motors_mixed = crate::tasks::controller_rate::RATE_MOTORS_MIXED.receiver();

    loop {
        rcv_motors_state.get_and(|state| state.is_armed()).await;
        debug!("[if_estimator] Vehicle is armed, continuing");

        let force_target = rcv_motors_mixed.changed().await;
        let imu_data = rcv_imu_data.get().await;

        let accel: f32 = imu_data.acc[2].abs();
        let force: f32 = force_target.iter().sum();

        debug!("[if_estimator] Accel {}, force: {}", accel, force);

        if force.abs() > VEHICLE_MASS * GRAVITY && accel > GRAVITY {
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

const VEHICLE_MASS: f32 = 0.566;
