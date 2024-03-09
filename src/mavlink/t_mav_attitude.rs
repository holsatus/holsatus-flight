use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    signal::Signal,
};
use embassy_time::{Duration, Ticker};
use mavlink::common::{MavMessage, ATTITUDE_DATA};

use crate::functions::time_boot_ms;
use crate::messaging as msg;

pub(super) static MAV_FREQUENCY: Signal<ThreadModeRawMutex, Option<Duration>> = Signal::new();

/// Task to handle sending timed Mavlink messages
#[embassy_executor::task]
pub async fn mav_attitude() -> ! {

    // Input messages
    let mut rcv_attitude = msg::ATTITUDE_EULER.receiver().unwrap();
    let mut rcv_imu = msg::IMU_DATA.receiver().unwrap();

    // Output messages
    let pub_message = super::MAV_MSG_QUEUE.sender();

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

            // Fetch latest attitude message
            let attitude = rcv_attitude.get().await;
            let rates = rcv_imu.get().await.gyr;

            // Construct message
            let msg = ATTITUDE_DATA {
                time_boot_ms: time_boot_ms(),
                roll: attitude.x,
                pitch: attitude.y,
                yaw: attitude.z,
                rollspeed: rates.x,
                pitchspeed: rates.y,
                yawspeed: rates.z,
            };

            // Put message into Mavlink queue
            pub_message.send(MavMessage::ATTITUDE(msg)).await;
        }
    }
}
