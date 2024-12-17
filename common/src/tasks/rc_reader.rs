use embassy_time::{Duration, Instant};
use embedded_io::Error;

use crate::{
    parsers::{crsf::CrsfParser, sbus::SbusParser}, signals as s, types::status::RcStatus
};

pub enum Parser {
    Sbus(SbusParser),
    Crsf(CrsfParser),
}

pub async fn main(mut uart_rx: impl embedded_io_async::Read) -> ! {
    const ID: &str = "rc_serial_reader";
    info!("{}: Task started", ID);

    // Output signals
    let snd_rc_channels = s::RC_CHANNELS_RAW.sender();
    let snd_rc_status = s::RC_STATUS.sender();

    let mut parser = Parser::Crsf(CrsfParser::new());
    let mut buffer = [0u8; 64];

    // Used to avoid excessive logging for some Uart errors
    let mut err_debounce = Instant::now();

    // Indicates the instant when the last packet was parsed
    let mut last_parse_time = Instant::now();

    // Assume worst case initial values
    let mut rc_status = RcStatus {
        failsafe: true,
        quality: 0,
    };

    info!("{}: Entering main loop", ID);
    'reading: loop {

        let fail = last_parse_time.elapsed() > Duration::from_millis(250);
        if rc_status.failsafe != fail {
            rc_status.failsafe = fail;
            snd_rc_status.send(rc_status);
        }

        // Read serial data, but with a timeout
        let bytes = match uart_rx.read(&mut buffer).await {
                Ok(bytes) => bytes,
                Err(err) => {
                    if err_debounce.elapsed() > Duration::from_millis(50) {
                        err_debounce = Instant::now();
                        error!("{}: Failed to read serial data: {:?}", ID, err.kind());
                    }
                    continue 'reading;
            },
        };

        // Exhaustively parse bytes. This might immediately overwrite
        // a just-parsed packet, but it ensures we use the latest packet.
        let mut sub_buffer = &buffer[..bytes];
        while sub_buffer.len() > 0 {
            match &mut parser {
                Parser::Sbus(sbus) => {
                    let result;
                    (result, sub_buffer) = sbus.push_bytes(sub_buffer);

                    match result {
                        Some(Ok(packet)) if !packet.failsafe => {
                            rc_status.failsafe = false;
                            last_parse_time = Instant::now();
                            snd_rc_channels.send(Some(packet.channels))
                        }
                        Some(Ok(_)) => {
                            rc_status.failsafe = true;
                            snd_rc_channels.send(None);
                            error!("{}: Sbus failsafe active", ID)
                        }
                        Some(Err(e)) => error!("{}: Error parsing: {:?}", ID, e),
                        None => continue 'reading, // The buffer is empty
                    }
                }
                Parser::Crsf(crsf) => {
                    let result;
                    (result, sub_buffer) = crsf.push_bytes(sub_buffer);


                    match result {
                        Some(Ok(raw_packet)) => {
                            rc_status.failsafe = false;
                            last_parse_time = Instant::now();
                            use crate::parsers::crsf::packet_containers::Packet;
                            match raw_packet.to_packet() {
                                Ok(Packet::RcChannelsPacked(packet)) => {
                                    snd_rc_channels.send(Some(packet.0))
                                }
                                Ok(Packet::LinkStatistics(packet)) => {
                                    rc_status.quality = packet.uplink_link_quality;
                                    snd_rc_status.send(rc_status);
                                }
                                _ => error!("{}: crsf packet not supported by Holsatus", ID),
                            }
                        },
                        None | Some(Err(crate::parsers::crsf::Error::NoSyncByte)) => continue 'reading, // The buffer is empty
                        Some(Err(err)) => {
                            crsf.reset();
                            error!("{}: Error parsing: {:?}", ID, err);
                            continue 'reading;
                        },
                    }
                }
            }
        }
    }
}