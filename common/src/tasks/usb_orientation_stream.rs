use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::Write;
use ufmt::uwrite;

use crate::{
    errors::adapter::embedded_io::EmbeddedIoError,
    serial::IoStream,
    signals,
    utils::u_types::{UBuffer, UFloat},
};

const STREAM_HZ: u64 = 50;

#[embassy_executor::task]
pub async fn main(serial_id: &'static str) -> ! {
    let mut serial = crate::serial::claim(serial_id).unwrap();
    let mut rcv_usb_connected = signals::USB_CONNECTED.receiver();

    loop {
        // Wait until the host opens the USB CDC connection.
        let _ = rcv_usb_connected.get_and(|&connected| connected).await;

        if let Err(error) = run_stream(&mut serial).await {
            match error {
                EmbeddedIoError::NotConnected | EmbeddedIoError::TimedOut => {
                    Timer::after_millis(100).await;
                }
                _ => {
                    error!("[usb-orientation-stream] serial error: {:?}", error);
                    Timer::after_millis(100).await;
                }
            }
        }
    }
}

async fn run_stream(serial: &mut IoStream) -> Result<(), EmbeddedIoError> {
    let mut ticker = Ticker::every(Duration::from_hz(STREAM_HZ));

    // CSV header: timestamp and attitude in radians.
    serial
        .write_all(b"t_ms,roll,pitch,yaw,qw,qx,qy,qz\n")
        .await?;

    loop {
        ticker.next().await;

        let Some(att_q) = signals::AHRS_ATTITUDE_Q.try_get() else {
            continue;
        };

        let (roll, pitch, yaw) = att_q.euler_angles();
        let q = att_q.quaternion();

        let mut buffer = UBuffer::<192>::new();
        uwrite!(
            buffer,
            "{},{},{},{},{},{},{},{}\n",
            Instant::now().as_millis(),
            UFloat(roll, 6),
            UFloat(pitch, 6),
            UFloat(yaw, 6),
            UFloat(q.w, 6),
            UFloat(q.i, 6),
            UFloat(q.j, 6),
            UFloat(q.k, 6)
        )
        .map_err(EmbeddedIoError::from)?;

        serial.write_all(buffer.bytes()).await?;
        serial.flush().await?;
    }
}
