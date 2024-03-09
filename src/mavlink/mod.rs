use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{AnyPin, Output},
    peripherals::UART0,
    uart::{self, Async, Uart, UartRx, UartTx},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, DynamicReceiver, DynamicSender},
};
use embassy_time::Duration;
use mavlink::{
    common::MavMessage, MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MAV_STX_V2,
};

use defmt::*;

use crate::{
    config::Configuration, functions::time_boot_ms, mavlink::supported_msg::MavStreamable,
};

mod t_mav_attitude;
mod t_mav_heartbeat;
mod t_mav_rc_channels;
mod t_mav_scaled_imu;

pub mod supported_msg;

pub type MavlinkUart = Uart<'static, UART0, Async>;
pub type MavlinkUartRx = UartRx<'static, UART0, Async>;
pub type MavlinkUartTx = UartTx<'static, UART0, Async>;

/// This channel is used to send messages to the Mavlink transmitter task.
/// It is a queue with backpressure, such that all messages sent to it are guaranteed to be sent to the transmitter task.
pub static MAV_MSG_QUEUE: Channel<CriticalSectionRawMutex, MavMessage, 5> = Channel::new();

pub fn mavlink_uart_companion_server(
    spawner: Spawner,
    uart_mavlink: MavlinkUart,
    led_pin: AnyPin,
    config: &'static Configuration,
) -> () {
    // Split mavlink UART stream into individual components
    let (uart_tx, uart_rx) = uart_mavlink.split();

    // Set default streaming frequencies according to config
    t_mav_attitude::MAV_FREQUENCY.signal(config.mav_freq.attitude);
    t_mav_heartbeat::MAV_FREQUENCY.signal(config.mav_freq.heartbeat);
    t_mav_scaled_imu::MAV_FREQUENCY.signal(config.mav_freq.scaled_imu);
    t_mav_rc_channels::MAV_FREQUENCY.signal(config.mav_freq.rc_channels);

    // Spawn mavlink streaming tasks
    spawner.must_spawn(t_mav_attitude::mav_attitude());
    spawner.must_spawn(t_mav_heartbeat::mav_heartbeat());
    spawner.must_spawn(t_mav_scaled_imu::mav_scaled_imu());
    spawner.must_spawn(t_mav_rc_channels::mav_rc_channels());

    // Spawn uart TX service
    spawner.must_spawn(mavlink_tx_server(
        uart_tx,
        MAV_MSG_QUEUE.receiver().into(),
        led_pin,
    ));

    // Spawn uart RX service
    spawner.must_spawn(mavlink_rx_server(uart_rx));
}

/// Server to handle transmitting messages onto UART line
#[embassy_executor::task]
pub async fn mavlink_tx_server(
    mut uart_tx: MavlinkUartTx,
    sub_message: DynamicReceiver<'static, MavMessage>,
    led_pin: AnyPin,
) {
    // Setup LED status blinker
    let mut led = Output::new(led_pin, embassy_rp::gpio::Level::Low);

    // Standard header
    let mut header = MavHeader {
        system_id: 1,    // ID of system (eg. for swarms/fleets)
        component_id: 1, // ID of component: 1 is from autopilot
        sequence: 0,     // Counted number of messages
    };

    info!("[MAVLINK-TX] Entering main loop");
    loop {
        // Receive mavlink message from queue
        let message = sub_message.receive().await;

        // Toggle LED on heartbeat message sent
        if let MavMessage::HEARTBEAT(_) = message {
            led.toggle();
        }

        // Transmit mavlink message to uart
        if let Err(e) = mavlink::write_v2_msg(&mut uart_tx, header, &message) {
            error!("[MAVLINK-TX] Mavlink uart write error {}", Debug2Format(&e));
        }

        // Echo Mav message to defmt output
        // println!("MavMessage: {:?}", defmt::Debug2Format(&message.as_mav_message()));

        // Increment message sequence counter
        header.sequence = header.sequence.wrapping_add(1);
    }
}

/// Server to handle receiving mavlink messages from UART
#[embassy_executor::task]
pub async fn mavlink_rx_server(mut uart_rx: MavlinkUartRx) {
    let mav_msg_pub = MAV_MSG_QUEUE.sender().into();

    info!("[MAVLINK-RX] Entering main loop");
    '_infinite: loop {
        let Ok((header, mav_msg)) = async_mavlink_read_v2_msg::<MavMessage>(&mut uart_rx).await
        else {
            error!("[MAVLINK-RX] Error receiving mavlinkerror!");
            continue;
        };
        // TODO Send mavlink response to unknown message type

        match mav_msg {
            MavMessage::HEARTBEAT(_heartbeat) => {
                info!("[MAVLINK-RX] Received heartbeat message");
            }
            MavMessage::PING(ping) => handle_ping(&mav_msg_pub, ping, header).await,
            MavMessage::SYSTEM_TIME(time) => handle_system_time(&mav_msg_pub, time).await,
            MavMessage::COMMAND_INT(command_int) => handle_command_int(command_int),
            MavMessage::ATTITUDE(_) => {}
            MavMessage::GPS_RAW_INT(_) => {}
            MavMessage::SCALED_IMU(_) => {}
            MavMessage::RC_CHANNELS(_) => {}
            _ => {
                warn!("[MAVLINK-RX] Received unknown message type")
            }
        }
    }
}

async fn handle_system_time(
    mav_msg_pub: &DynamicSender<'static, MavMessage>,
    time: mavlink::common::SYSTEM_TIME_DATA,
) {
    // System time microservice
    mav_msg_pub
        .send(MavMessage::SYSTEM_TIME(mavlink::common::SYSTEM_TIME_DATA {
            time_unix_usec: time.time_unix_usec,
            time_boot_ms: time_boot_ms(),
        }))
        .await;
}

async fn handle_ping(
    mav_msg_pub: &DynamicSender<'static, MavMessage>,
    ping: mavlink::common::PING_DATA,
    header: MavHeader,
) {
    // Ping microservice implemented as documented in
    // https://mavlink.io/en/services/ping.html
    mav_msg_pub
        .send(MavMessage::PING(mavlink::common::PING_DATA {
            time_usec: ping.time_usec,
            seq: ping.seq,
            target_system: header.system_id,
            target_component: header.component_id,
        }))
        .await;
}

fn handle_command_int(command_int: mavlink::common::COMMAND_INT_DATA) {
    match command_int.command {
        mavlink::common::MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
            handle_mav_cmd_set_message_interval(command_int);
        }
        _ => { /* Remaining commands are not implemented */ }
    }
}

fn handle_mav_cmd_set_message_interval(
    command_int: mavlink::common::COMMAND_INT_DATA,
) -> Option<()> {
    let interval = if command_int.param2 < 1. {
        None
    } else {
        Some(Duration::from_micros(command_int.param2 as u64))
    };

    match MavStreamable::from_id(command_int.param1 as u32)? {
        MavStreamable::Attitude => t_mav_attitude::MAV_FREQUENCY.signal(interval),
        MavStreamable::Heartbeat => t_mav_heartbeat::MAV_FREQUENCY.signal(interval),
        MavStreamable::ScaledImu => t_mav_scaled_imu::MAV_FREQUENCY.signal(interval),
        MavStreamable::RcChannels => t_mav_rc_channels::MAV_FREQUENCY.signal(interval),
        _ => { /* Remaining commands are not implemented */ }
    }

    Some(())
}

/// Read a MAVLink v2  message from an async uart stream
pub async fn async_mavlink_read_v2_msg<'a, M: Message>(
    reader: &mut MavlinkUartRx,
) -> Result<(MavHeader, M), uart::Error> {
    loop {
        let message = async_mavlink_read_v2_raw_message(reader).await?;

        // bad crc: ignore message
        if !message.has_valid_crc::<M>() {
            continue;
        }

        let header = MavHeader {
            sequence: message.sequence(),
            system_id: message.system_id(),
            component_id: message.component_id(),
        };

        let msg = M::parse(MavlinkVersion::V2, message.message_id(), message.payload()).map_err(|_|uart::Error::Framing)?;

        return Ok((header, msg));
    }
}

/// Return a raw buffer with the mavlink message
/// V2 maximum size is 280 bytes: `<https://mavlink.io/en/guide/serialization.html>`
pub async fn async_mavlink_read_v2_raw_message<'a>(
    reader: &mut MavlinkUartRx,
) -> Result<MAVLinkV2MessageRaw, uart::Error> {
    let magic_byte: u8 = 0;

    // search for the magic framing value indicating start of mavlink message
    while !(reader.read(&mut [magic_byte]).await.is_ok() && magic_byte == MAV_STX_V2) {}

    let mut message = MAVLinkV2MessageRaw::new();

    *message.mut_magic() = MAV_STX_V2;
    reader.read(message.mut_header()).await?;
    reader.read(message.mut_payload_and_checksum_and_sign()).await?;

    Ok(message)
}
