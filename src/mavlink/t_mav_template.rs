use embassy_sync::{signal::Signal, blocking_mutex::raw::ThreadModeRawMutex};
use embassy_time::{Ticker, Duration};
use mavlink::common::HEARTBEAT_DATA;

use super::supported_msg::HolsatusMavMsg;

/// Task to handle sending timed Mavlink messages
#[embassy_executor::task]
pub async fn mav_template(
    pub_message: DynSubscriber<'static, HolsatusMavMsg>,
    sub_duration: &'static Signal<ThreadModeRawMutex,Option<Duration>>
) {

    loop {

        // Wait for signal of message duration (frequency)
        let mut duration_option = sub_duration.wait().await ;
        let Some(duration) = duration_option else {continue};
        let mut ticker = Ticker::every(duration);

        while duration_option.is_some() {

            // Construct heartbeat message
            let msg = HEARTBEAT_DATA{
                custom_mode: 0,
                mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
                autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
                base_mode: mavlink::common::MavModeFlag::empty(),
                system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0x3,
            };

            // Signal message
            pub_message.publish(HolsatusMavMsg::Heartbeat(msg)).await;
            
            // Wait for next timed tick
            ticker.next().await;
            
            // Check for new ticker value
            if sub_duration.signaled() {
                let duration_received = sub_duration.wait().await;
                if duration_option != duration_received {
                    duration_option = duration_received;
                    let Some(duration) = duration_option else {continue};
                    ticker = Ticker::every(duration)
                }
            }
        }
    }
}