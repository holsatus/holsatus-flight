
use embassy_rp::{
    peripherals::UART1,
    uart::{Async, UartRx},
};
use embassy_time::{with_timeout, Duration, Instant};
use sbus::SBusPacketParser;

use crate::messaging as msg;

#[derive(Clone, Copy, Debug)]
pub enum RxError {
    /// The MCU Serial interface reported a read error.
    SerialRead,

    /// No Serial data was received within the timeout period.
    SerialTimeout,

    /// It took too long to parse a valid packet.
    /// This could mean the packet is malformed,
    /// a hardware issue with the receiver or similar.
    ParseTimeout,

    /// The packet was parsed, but the failsafe was active.
    /// This is the expected condition if the transmitter is
    /// turned off or the signal is lost.
    Failsafe,
}

/// Task to read sbus data from UART1.
///
/// Utilizes a custom interrupt-based UART driver
/// to read entire SBUS packet in one go.
#[embassy_executor::task]
pub async fn sbus_reader(mut uart_rx_sbus: UartRx<'static, UART1, Async>, timeout: Duration) {
    // Output messages
    let snd_sbus_data = msg::SBUS_DATA.dyn_sender();

    let mut read_buffer = [0u8; 25];
    let mut parser = SBusPacketParser::new();
    let mut parse_time = Instant::now();

    '_infinite: loop {
        match with_timeout(timeout, uart_rx_sbus.read_fifo_on_timeout(&mut read_buffer)).await {
            Ok(Ok(bytes)) => {
                parser.push_bytes(&read_buffer[0..bytes]);

                // If packet was parsed with no failsafe, send it
                if let Some(packet) = parser.try_parse() {
                    parse_time = Instant::now();
                    match packet.failsafe {
                        false => snd_sbus_data.send(Ok(packet)),
                        true => snd_sbus_data.send(Err(RxError::Failsafe)),
                    }

                // Else if packet was not parsed, check if it took too long
                } else if parse_time.elapsed() > timeout {
                    snd_sbus_data.send(Err(RxError::ParseTimeout))
                }
            }
            Ok(Err(_)) => snd_sbus_data.send(Err(RxError::SerialRead)),
            Err(_) => snd_sbus_data.send(Err(RxError::SerialTimeout)),
        }
    }
}