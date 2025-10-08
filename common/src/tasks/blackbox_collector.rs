use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};

use crate::types::blackbox::get_rate_log;
use crate::types::blackbox::{get_angle_log, LoggableType};

use crate::errors::BlackboxError;
use crate::{get_or_warn, signals as s};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "blackbox_collector";

    info!("{}: Task started, entering main loop", ID);

    let mut rcv_log_preset = s::CFG_LOG_PRESET.receiver();

    let mut preset = get_or_warn!(rcv_log_preset).await;

    let mut ticker = Ticker::every(Duration::from_hz(preset.base_log_rate as u64));
    let mut counter = 0u8;
    loop {
        // Update the log preset of wait for the next logging tick
        if let Either::First(new_preset) = select(rcv_log_preset.changed(), ticker.next()).await {
            preset = new_preset;
            continue;
        }

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
