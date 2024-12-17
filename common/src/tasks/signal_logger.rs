
use crate::signals as s;
use embassy_time::{Duration, Ticker};

pub async fn main() -> ! {
    static STR_ID: &str = "signal_logger";
    info!("{}: Task started, entering main loop", STR_ID);

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        ticker.next().await;
        debug!("RAW_IMU_DATA: {:?}", s::RAW_IMU_DATA.try_get());
        debug!("CAL_IMU_DATA: {:?}", s::CAL_IMU_DATA.try_get());
        debug!("CONTROL_MODE: {:?}", s::CONTROL_MODE.try_get());
        debug!("CTROL_MOTORS: {:?}", s::CTRL_MOTORS.try_get());
        debug!("ANGLE_SP: {:?}", s::VEL_TO_ANGLE_SP.try_get());
        debug!("RATE_SP: {:?}", s::ANGLE_TO_RATE_SP.try_get());
        debug!("RC_CONTROLS: {:?}", s::RC_ANALOG_RATE.try_get());
    }
}
