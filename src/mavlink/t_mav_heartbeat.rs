use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    signal::Signal,
};
use embassy_time::{Duration, Ticker};
use mavlink::common::{self, MavMessage};
use crate::messaging as msg;

pub(super) static MAV_FREQUENCY: Signal<ThreadModeRawMutex, Option<Duration>> = Signal::new();

/// Task to handle sending timed Mavlink messages
#[embassy_executor::task]
pub async fn mav_heartbeat() -> ! {
    // Output messages
    let pub_message = super::MAV_MSG_QUEUE.sender();

    // TODO Pull this form live vehicle state
    let mut rcv_vehicle_state = msg::VEHICLE_STATE.receiver().unwrap();

    'infinite: loop {
        // Wait for signal of message duration (frequency)
        let Some(duration) = MAV_FREQUENCY.wait().await else {
            continue 'infinite;
        };
        let mut ticker = Ticker::every(duration);

        'periodic: loop {
            // Wait for tick or change in frequency
            match select(MAV_FREQUENCY.wait(), ticker.next()).await {
                First(Some(d)) => ticker = Ticker::every(d),
                First(None) => break 'periodic,
                Second(()) => { /* ticker activated */ }
            }

            let vehicle_state = rcv_vehicle_state.changed().await;

            // Construct heartbeat message
            let msg = common::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: common::MavType::MAV_TYPE_QUADROTOR,
                autopilot: common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: common::MavModeFlag::empty(),
                system_status: vehicle_state.clone().into(),
                mavlink_version: 0x3,
            };

            // Signal message
            pub_message.send(MavMessage::HEARTBEAT(msg)).await;
        }
    }
}
