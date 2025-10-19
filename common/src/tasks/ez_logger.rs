use crate::{signals as s, tasks::commander::CMD_CONTROL_MODE};
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "ez_logger";
    info!("{}: Task started, entering main loop", STR_ID);

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        ticker.next().await;
        debug!("RAW_IMU_DATA: {:?}", s::RAW_IMU_DATA.try_get());
        debug!("CAL_IMU_DATA: {:?}", s::CAL_IMU_DATA.try_get());
        debug!("CONTROL_MODE: {:?}", CMD_CONTROL_MODE.try_get());
        debug!("CTROL_MOTORS: {:?}", s::CTRL_MOTORS.try_get());
        debug!(
            "ANGLE_SP: {:?}",
            s::VEL_TO_ANGLE_SP
                .try_get()
                .map(|q| q.as_vector().clone_owned().data.0)
        );
        debug!("RATE_SP: {:?}", s::ANGLE_TO_RATE_SP.try_get());
        debug!("RC_CONTROLS: {:?}", s::RC_ANALOG_UNIT.try_get());
    }
}
