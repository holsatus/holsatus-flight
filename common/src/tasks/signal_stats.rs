
use crate::signals as s;
use embassy_time::{Duration, Ticker};

macro_rules! show_rate {
    
    ($($signal:ident$([$idx:literal])? => $prev:ident),* $(,)?) => {
        // Initialize previous message IDs for each signal
        $(
            let mut $prev = s::$signal$([$idx])?.get_msg_id();
        )*

        let mut ticker = Ticker::every(Duration::from_hz(1));

        let mut temp;

        loop {
            ticker.next().await;

            info!("pipe rate: {:?} us", s::RATE_PIPELINE_TIME.try_get().map(|t|t.map(|i|i.as_micros())));

            // Calculate and log the rate for each signal
            $(
                temp = s::$signal$([$idx])?.get_msg_id();
                info!("{}: {:?} msg/s", stringify!($signal$([$idx])?), temp - $prev);
                $prev = temp;
            )*
        }
    };

}

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "signal_logger";
    info!("{}: Task started, entering main loop", STR_ID);

    show_rate!(
        RAW_MULTI_IMU_DATA[0] => prev_raw_imu_data_0,
        CAL_MULTI_IMU_DATA[0] => prev_cal_imu_data_0,
        RAW_MULTI_IMU_DATA[1] => prev_raw_imu_data_1,
        CAL_MULTI_IMU_DATA[1] => prev_cal_imu_data_1,
        CAL_IMU_DATA => prev_cal_imu_data,
        TRUE_RATE_SP => prev_true_rate_sp,
        MOTORS_STATE => prev_motors_state,
        CTRL_MOTORS => prev_ctrl_motors,
    );
}
