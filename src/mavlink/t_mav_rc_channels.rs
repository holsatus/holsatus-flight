use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    signal::Signal,
};
use embassy_time::{Duration, Ticker};
use mavlink::common::{MavMessage, RC_CHANNELS_DATA};

use crate::functions::time_boot_ms;
use crate::messaging as msg;

pub(super) static MAV_FREQUENCY: Signal<ThreadModeRawMutex, Option<Duration>> = Signal::new();

/// Task to handle sending timed Mavlink messages
#[embassy_executor::task]
pub async fn mav_rc_channels() -> ! {
    // Input messages
    let mut rcv_rc_channels = msg::SBUS_DATA.receiver().unwrap();

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

            // Fetch latest rc message
            if let Ok(packet) = rcv_rc_channels.get().await {
                // Construct message
                let msg = RC_CHANNELS_DATA {
                    time_boot_ms: time_boot_ms(),
                    chan1_raw: packet.channels[0],
                    chan2_raw: packet.channels[1],
                    chan3_raw: packet.channels[2],
                    chan4_raw: packet.channels[3],
                    chan5_raw: packet.channels[4],
                    chan6_raw: packet.channels[5],
                    chan7_raw: packet.channels[6],
                    chan8_raw: packet.channels[7],
                    chan9_raw: packet.channels[8],
                    chan10_raw: packet.channels[9],
                    chan11_raw: packet.channels[10],
                    chan12_raw: packet.channels[11],
                    chan13_raw: packet.channels[12],
                    chan14_raw: packet.channels[13],
                    chan15_raw: packet.channels[14],
                    chan16_raw: packet.channels[15],
                    chan17_raw: 0,
                    chan18_raw: 0,
                    chancount: 0,
                    rssi: 0, // TODO implement RSSI
                };

                // Put message into Mavlink queue
                pub_message.send(MavMessage::RC_CHANNELS(msg)).await;
            };
        }
    }
}
