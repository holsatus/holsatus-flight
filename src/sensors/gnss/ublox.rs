use embassy_rp::{
    peripherals::UART1,
    uart::{Async, UartRx},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, pubsub::Publisher};
use ublox::{FixedLinearBuffer, Parser};

#[derive(Default, Clone, Copy)]
#[allow(unused)]
pub struct NavPosLlh {
    /// GPS Millisecond Time of Week
    itow: u32,

    /// Longitude
    lon_deg: f64,

    /// Latitude
    lat_deg: f64,

    /// Height above Ellipsoid
    height_meters: f64,

    /// Height above mean sea level
    height_msl: f64,

    /// Horizontal Accuracy Estimate
    h_ack: f64,

    /// Vertical Accuracy Estimate
    v_acc: f64,
}

#[allow(unused)]
async fn ublox_reader(
    mut uart: UartRx<'_, UART1, Async>,
    tx: Publisher<'static, ThreadModeRawMutex, NavPosLlh, 1, 1, 1>,
) {
    let mut data: NavPosLlh;

    let mut buf = [0; 64];
    let buf = FixedLinearBuffer::new(&mut buf);

    let mut parser = Parser::new(buf);
    let mut read_byte: [u8; 1] = [0];

    loop {
        if uart.read(&mut read_byte).await.is_ok() {
            let mut iter = parser.consume(&read_byte);
            if let Some(Ok(ublox::PacketRef::NavPosLlh(navpos))) = iter.next() {
                data = NavPosLlh {
                    itow: navpos.itow(),
                    lat_deg: navpos.lat_degrees(),
                    lon_deg: navpos.lon_degrees(),
                    height_meters: navpos.height_meters(),
                    height_msl: navpos.height_msl(),
                    h_ack: navpos.h_ack(),
                    v_acc: navpos.v_acc(),
                };
                tx.publish_immediate(data);
            }
        }
    }
}
