use embassy_time::{with_timeout, Duration};
use ublox::{proto23::PacketRef, FixedBuffer, Parser, UbxPacket};

#[embassy_executor::task]
pub async fn main(stream_id: &'static str) -> ! {
    const ID: &str = "gnss_reader";
    info!("{}: Task started", ID);

    let mut stream = crate::serial::claim(stream_id).unwrap();
    let mut snd_raw_gnss_data = crate::signals::RAW_GNSS_DATA.sender();
    let mut parser: Parser<FixedBuffer<128>> = Parser::new_fixed();

    info!("{}: Entering main loop", ID);
    'parsing: loop {
        // Read serial data, but with a timeout
        let received_bytes =
            match with_timeout(Duration::from_secs(1), stream.reader.fill_buf()).await {
                Ok(Ok(bytes)) => bytes,
                Ok(Err(_)) => {
                    error!("{}: Failed to read serial data", ID);
                    continue 'parsing;
                }
                Err(_) => {
                    warn!("{}: Timeout while reading serial data", ID);
                    continue 'parsing;
                }
            };

        // Exhaustively parse bytes. This might immediately overwrite
        // a just-parsed packet, but it ensures we use the latest packet.
        let num_received_bytes = received_bytes.len();
        let mut ubx_parser_iter = parser.consume_ubx(received_bytes);

        while let Some(parsed) = ubx_parser_iter.next() {
            match parsed {
                Ok(packet) => match packet {
                    UbxPacket::Proto23(PacketRef::NavPvt(pvt)) => {
                        snd_raw_gnss_data.send(pvt.into());
                    }
                    _ => error!("[gnss_reader] Unknown UBX protocol or packet type"),
                },
                Err(_) => error!("[gnss_reader] Error parsing UBX packet"),
            }
        }

        core::mem::drop(ubx_parser_iter);
        stream.reader.consume(num_received_bytes);
    }
}
