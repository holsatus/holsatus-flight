use embassy_time::{with_timeout, Duration, Instant};

use crate::types::measurements::{GnssData, GnssTime};

#[embassy_executor::task]
pub async fn main(stream_id: &'static str) -> ! {
    const ID: &str = "gnss_reader";
    info!("{}: Task started", ID);

    let mut stream = crate::serial::claim(stream_id).unwrap();

    let mut snd_raw_gnss_data = crate::signals::RAW_GNSS_DATA.sender();

    let mut buffer = [0u8; 128];
    let mut parse_buffer = [0u8; 128];
    let buf = ublox::FixedLinearBuffer::new(&mut parse_buffer);
    let mut parser = ublox::Parser::new(buf);

    info!("{}: Entering main loop", ID);
    'parsing: loop {
        // Read serial data, but with a timeout
        let bytes =
            match with_timeout(Duration::from_secs(1), stream.reader.read(&mut buffer)).await {
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

        let sub_buffer = &buffer[..bytes];
        let mut consumed = parser.consume(sub_buffer);

        while let Some(parsed) = consumed.next() {
            match parsed {
                Ok(packet) => match packet {
                    ublox::PacketRef::NavPvt(pvt) => {
                        let time = GnssTime {
                            year: pvt.year(),
                            month: pvt.month(),
                            day: pvt.day(),
                            hour: pvt.hour(),
                            min: pvt.min(),
                            sec: pvt.sec(),
                        };

                        let gnss_packet = GnssData {
                            timestamp_us: Instant::now().as_micros(),
                            time,
                            fix: pvt.fix_type().into(),
                            satellites: pvt.num_satellites(),
                            lat_raw: pvt.lat_degrees_raw(),
                            lon_raw: pvt.lon_degrees_raw(),
                            altitude: pvt.height_meters() as f32,
                            horiz_accuracy: pvt.horiz_accuracy() as f32 * 1e-3,
                            vert_accuracy: pvt.vert_accuracy() as f32 * 1e-3,
                            vel_north: pvt.vel_north() as f32,
                            vel_east: pvt.vel_east() as f32,
                            vel_down: pvt.vel_down() as f32,
                            vel_accuracy: pvt.speed_accuracy_estimate() as f32,
                            vel_ground: pvt.ground_speed() as f32,
                            heading: pvt.heading_degrees() as f32,
                            heading_accuracy: pvt.heading_accuracy_estimate() as f32,
                            mag_declination: pvt.magnetic_declination_degrees() as f32,
                        };

                        snd_raw_gnss_data.send(gnss_packet);
                    }
                    _ => error!("{}: Unknown GNSS packet type", ID),
                },
                Err(_) => error!("{}: Error parsing GNSS packet", ID),
            }
        }
    }
}
