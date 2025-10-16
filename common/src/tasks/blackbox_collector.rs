use embassy_time::{Duration, Ticker};

use crate::types::blackbox::{get_angle_log, LoggableType};
use crate::types::blackbox::{get_rate_log, LogPreset};

use crate::errors::BlackboxError;
use crate::signals as s;

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "blackbox_collector";

    info!("{}: Task started, entering main loop", ID);

    let preset = LogPreset::default();

    let mut ticker = Ticker::every(Duration::from_hz(preset.base_log_rate as u64));
    let mut counter = 0u8;
    loop {
        // Update the log preset of wait for the next logging tick
        ticker.next().await;

        // RATE LOG
        if preset.rate_div.should_run_balanced(counter) {
            if let Some(log) = get_rate_log() {
                s::BLACKBOX_QUEUE.send(LoggableType::Rate(log)).await;
            } else {
                crate::types::blackbox::log_error(BlackboxError::MessageFormingError);
            }
        }

        // ANGLE LOG
        if preset.angle_div.should_run_balanced(counter) {
            if let Some(log) = get_angle_log() {
                s::BLACKBOX_QUEUE.send(LoggableType::Angle(log)).await;
            } else {
                crate::types::blackbox::log_error(BlackboxError::MessageFormingError);
            }
        }

        // TODO: Not yet implemented
        // if preset.velocity_div.should_run_balanced(counter) {
        //     if let Some(log) = get_angle_log() {
        //         s::BLACKBOX_QUEUE.send(LoggableType::Angle(log)).await;
        //     }
        // }

        // TODO: Not yet implemented
        // if preset.position_div.should_run_balanced(counter) {
        //     if let Some(log) = get_angle_log() {
        //         s::BLACKBOX_QUEUE.send(LoggableType::Angle(log)).await;
        //     }
        // }

        counter = counter.wrapping_add(1);
    }
}
