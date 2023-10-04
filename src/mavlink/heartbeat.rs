use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Ticker, Duration};
use mavlink::common::MavMessage;
use crate::channels::*;

/// Channel for updating the message interval for heartbeat messages
pub static INTERVAL: super::Ch<Option<Duration>,1> = PubSubChannel::new();

/// Task to handle sending timed Mavlink heartbeat messages
#[embassy_executor::task]
pub async fn mav_heartbeat(
    mut sub_interval: Sub<Option<Duration>,1>,
    pub_message: super::MavlinkMsgPub
) {

    let mut ticker: Ticker;
    let mut mav_subtask_active: bool;

    loop {

        // Check if subtask should be active
        match sub_interval.next_message_pure().await {
            Some(interval) => {
                ticker = Ticker::every(interval);
                mav_subtask_active = true;
    
            },
            None => {
                ticker = Ticker::every(Duration::from_hz(1));
                mav_subtask_active = false;
    
            }
        };

        // Keep subtask active
        while mav_subtask_active {

            // Construct heartbeat message
            let msg_heartbeat = MavMessage::HEARTBEAT(
                mavlink::common::HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
                    autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
                    base_mode: mavlink::common::MavModeFlag::empty(),
                    system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
                    mavlink_version: 0x3,
                }
            );

            // Publish and wait if message queue is full
            pub_message.publish(msg_heartbeat).await;

            // Check for updates to interval ticker
            if let Some(message) = sub_interval.try_next_message_pure() {
                match message {
                    Some(duration) => ticker = Ticker::every(duration),
                    None => mav_subtask_active = false,
                }
            }
            
            ticker.next().await;
            
        }
    }
}