use defmt::*;
use embassy_rp::{
    peripherals::UART1,
    uart::{self, Async},
};
use embassy_time::{with_timeout, Instant};
use sbus::SBusPacketParser;

use crate::cfg;
use crate::channels;
use crate::sbus_cmd;

static TASK_ID: &str = "[SBUS_READER]";

#[embassy_executor::task]
pub async fn sbus_reader(
    mut uart_rx_sbus: uart::UartRx<'static, UART1, Async>,
    p_sbus_cmd: channels::SbusCmdPub,
) {
    let mut parser = SBusPacketParser::new();
    let mut prev_parse_time = Instant::now();

    // A buffer length of 1 may be slower, but perhaps also reduces
    // the risk of the buffer coming out of sync with the message..
    let mut read_buffer = [0; 1];

    info!("{}: Entering main loop", TASK_ID);
    loop {
        match with_timeout(cfg::SBUS_PARSE_TIMEOUT, uart_rx_sbus.read(&mut read_buffer)).await {
            Ok(Ok(())) => {
                parser.push_bytes(&read_buffer);
                if let Some(packet) = parser.try_parse() {
                    let parse_time = Instant::now();
                    match sbus_cmd::convert(&packet) {
                        Some(cmd) => p_sbus_cmd.publish_immediate(Ok(cmd)),
                        None => p_sbus_cmd.publish_immediate(Err(SbusError::SbusFailsafe)),
                    }
                    prev_parse_time = parse_time;
                } else if Instant::now().duration_since(prev_parse_time) > cfg::SBUS_PARSE_TIMEOUT {
                    p_sbus_cmd.publish_immediate(Err(SbusError::ParseTimeout))
                }
            }
            Ok(Err(_)) => p_sbus_cmd.publish_immediate(Err(SbusError::SerialRead)),
            Err(_) => p_sbus_cmd.publish_immediate(Err(SbusError::SerialTimeout)),
        }
    }
}

#[derive(Clone, Format)]
pub enum SbusError {
    ParseTimeout,
    SerialRead,
    SerialTimeout,
    SbusFailsafe,
}
