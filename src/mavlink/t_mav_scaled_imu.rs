use crate::functions::time_boot_ms;
use crate::messaging as msg;
use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    signal::Signal,
};
use embassy_time::{Duration, Ticker};
use mavlink::common::{MavMessage, SCALED_IMU_DATA};

pub static MAV_FREQUENCY: Signal<ThreadModeRawMutex, Option<Duration>> = Signal::new();

/// Task to handle sending timed Mavlink messages
#[embassy_executor::task]
pub async fn mav_scaled_imu() -> ! {
    // Input messages
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();
    let mut rcv_mag_data = msg::MAG_DATA.receiver().unwrap();

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

            // Fetch latest sensor data
            let imu = rcv_imu_data.get().await;
            let mag = rcv_mag_data.get().await;

            // Construct message
            let msg = SCALED_IMU_DATA {
                time_boot_ms: time_boot_ms(),
                xacc: (imu.acc.x * 1e3) as i16,
                yacc: (imu.acc.y * 1e3) as i16,
                zacc: (imu.acc.z * 1e3) as i16,
                xgyro: (imu.gyr.x * 1e6) as i16,
                ygyro: (imu.gyr.y * 1e6) as i16,
                zgyro: (imu.gyr.z * 1e6) as i16,
                xmag: (mag.x * 1e3) as i16,
                ymag: (mag.y * 1e3) as i16,
                zmag: (mag.z * 1e3) as i16,
            };

            // Put message into Mavlink queue
            pub_message.send(MavMessage::SCALED_IMU(msg)).await;
        }
    }
}
