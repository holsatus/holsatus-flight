use defmt::{println, Debug2Format};
use embassy_time::{Duration, Ticker};

use crate::messaging as msg;

#[embassy_executor::task]
pub async fn status_printer() -> ! {
    let mut rcv_arming_prevention = msg::ARM_BLOCKER.receiver().unwrap();
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();
    let mut rcv_attitude_euler = msg::ATTITUDE_EULER.receiver().unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    '_infinite: loop {
        println!("{:?}", Debug2Format(&rcv_arming_prevention.get().await));
        println!("Imu data: {:?}", Debug2Format(&rcv_imu_data.try_get()));
        println!("Attitude: {:?}", Debug2Format(&rcv_attitude_euler.try_get()));
        ticker.next().await;
    }
}
