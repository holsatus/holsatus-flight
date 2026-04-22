//! MicoAir H743v2 -- ELRS / CRSF receiver sniffer.
//!
//! Reads CRSF frames from a receiver (e.g. RadioMaster RP3) wired to
//! USART6 RX (PC7), decodes the 16 channels and LinkStatistics, and
//! prints a live, human-readable view to USART1 TX (PA9) for miniterm.
//!
//! Run miniterm on the USB-serial adapter connected to PA9/GND at 115200
//! to watch the stream. No TX flashing required -- the TX15 already
//! speaks CRSF via its internal ELRS module.
//!
//! # Hardware
//!   RC:    USART6: RX=PC7 (TX=PC6 unused)  420000 baud (CRSF)
//!   Debug: USART1: TX=PA9                  115200 baud
//!
//! # Wiring (RP3 <-> FC UART6 port)
//!   RP3 5V  -> FC 5V
//!   RP3 GND -> FC GND
//!   RP3 TX  -> FC PC7 (UART6 RX)
//!   RP3 RX  -> FC PC6 (UART6 TX)  [optional, only needed for telemetry back]
//!
//! # LEDs
//!   Green : steady after init
//!   Blue  : toggles each decoded RcChannelsPacked frame
//!   Red   : failsafe (no frame for >250 ms)

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_futures::select::{select, Either};
use embassy_stm32::usart::{Config as UartConfig, UartRx, UartTx};
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use common::parsers::crsf::packet_containers::Packet;
use common::parsers::crsf::CrsfParser;

const CRSF_BAUD: u32 = 420_000;
const DEBUG_BAUD: u32 = 115_200;

const FAILSAFE_TIMEOUT: Duration = Duration::from_millis(250);
const TICK_PERIOD: Duration = Duration::from_secs(10);

// CRSF channel indices (0-based) to watch for switch transitions.
// Channel 4 = AUX1 = CRSF CH5 (SE -> KILL)
// Channel 5 = AUX2 = CRSF CH6 (SF -> RESTART)
// Map SE -> CH5 and SF -> CH6 on the TX15 (Model Setup -> Mixes).
// The "action" string is printed as a loud banner whenever that switch
// moves, so the operator can build muscle memory before real flight.
const WATCHED: &[(usize, &str, &str)] = &[
    (4, "SE", "KILL"),
    (5, "SF", "RESTART"),
];

// CRSF channel calibration (Betaflight / ELRS convention, 11-bit values).
const CRSF_LOW: i32 = 172;    // -100%
const CRSF_MID: i32 = 992;    //    0%
const CRSF_HIGH: i32 = 1811;  // +100%

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});
// USART6 + DMA2_STREAM2 are bound at lib level in micoairh743v2::rc_kill::RcIrqs.
use micoairh743v2::rc_kill::RcIrqs;

/// Decode a 3-position switch from a raw CRSF channel value.
fn switch_pos(raw: u16) -> &'static str {
    let v = raw as i32;
    if v < (CRSF_LOW + CRSF_MID) / 2 {
        "LOW "
    } else if v > (CRSF_MID + CRSF_HIGH) / 2 {
        "HIGH"
    } else {
        "MID "
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    // Debug UART (USART1 TX for miniterm).
    let mut dbg_cfg = UartConfig::default();
    dbg_cfg.baudrate = DEBUG_BAUD;
    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, dbg_cfg).unwrap();
    uart.write(b"rc_data: USART1 TX ok\r\n").await.ok();

    // CRSF input (USART6 RX only). Uses lib-level RcIrqs so USART6 +
    // DMA2_STREAM2 are only bound once across all binaries.
    let mut rc_cfg = UartConfig::default();
    rc_cfg.baudrate = CRSF_BAUD;
    let mut rc_rx =
        UartRx::new(p.USART6, p.PC7, p.DMA2_CH2, RcIrqs, rc_cfg).unwrap();
    uart.write(b"rc_data: USART6 RX ok @420000\r\n").await.ok();
    uart.write(b"rc_data: waiting for CRSF frames...\r\n").await.ok();

    led_green.set_high();

    let mut parser = CrsfParser::new();
    let mut rx_buf = [0u8; 64];
    let mut channels: [u16; 16] = [CRSF_MID as u16; 16];
    let mut link_quality: u8 = 0;
    let mut have_channels = false;
    // Initialise last_packet far in the past so the initial state is
    // correctly "failsafe" (no packets yet), avoiding a startup false
    // positive LINK UP.
    let mut last_packet: Option<Instant> = None;
    let mut last_tick = Instant::now();
    let mut frame_count: u32 = 0;
    let mut byte_count: u32 = 0;
    let mut parse_err: u32 = 0;
    let mut uart_err: u32 = 0;
    let mut failsafe = true;

    // Remember previous switch positions for the watched channels only.
    // `""` means "unknown yet" (first packet triggers an init line).
    let mut last_switch: [&'static str; 16] = [""; 16];

    loop {
        // Run UART read and a 1 s heartbeat ticker concurrently.
        // Whichever fires first wins -- so we are guaranteed a status line
        // at least once per second regardless of RC link state.
        let reader = rc_rx.read_until_idle(&mut rx_buf);
        let ticker = Timer::after(Duration::from_secs(1));

        match select(reader, ticker).await {
            Either::First(Ok(n)) => {
                byte_count = byte_count.saturating_add(n as u32);
                process_bytes(
                    &rx_buf[..n],
                    &mut parser,
                    &mut channels,
                    &mut link_quality,
                    &mut have_channels,
                    &mut last_packet,
                    &mut frame_count,
                    &mut parse_err,
                    &mut led_blue,
                );
            }
            Either::First(Err(_)) => {
                uart_err = uart_err.saturating_add(1);
            }
            Either::Second(_) => {
                // Timer expired. Nothing to do here -- heartbeat below.
            }
        }

        // Failsafe edge detection (packet loss / recovery).
        let fs_now = match last_packet {
            Some(t) => t.elapsed() > FAILSAFE_TIMEOUT,
            None => true,
        };
        if fs_now != failsafe {
            failsafe = fs_now;
            if failsafe {
                led_red.set_high();
                uart.write(b"\r\n*** FAILSAFE: no CRSF frames ***\r\n").await.ok();
            } else {
                led_red.set_low();
                uart.write(b"\r\n*** LINK UP ***\r\n").await.ok();
            }
        }

        // Low-frequency heartbeat so miniterm shows the link is alive.
        if last_tick.elapsed() >= TICK_PERIOD {
            last_tick = Instant::now();
            let mut hb: String<128> = String::new();
            let _ = write!(
                hb,
                "tick up={}s frames={} lq={}% fs={}\r\n",
                Instant::now().as_secs(),
                frame_count,
                link_quality,
                failsafe as u8,
            );
            uart.write(hb.as_bytes()).await.ok();
            let _ = byte_count;
            let _ = parse_err;
            let _ = uart_err;
        }

        // Event-driven output: only print transitions on watched channels.
        // A transition prints a loud ACTION banner (KILL / RESTART) on the
        // line BELOW the status line so the habit is obvious on the screen.
        if have_channels {
            for &(idx, name, action) in WATCHED {
                let new_pos = switch_pos(channels[idx]);
                let old_pos = last_switch[idx];
                if old_pos == new_pos {
                    continue;
                }
                let mut line: String<160> = String::new();
                if old_pos.is_empty() {
                    let _ = write!(
                        line,
                        "{} ({}) init -> {} (raw={})\r\n",
                        name,
                        action,
                        new_pos.trim_end(),
                        channels[idx],
                    );
                } else {
                    let _ = write!(
                        line,
                        ">>> {} <<<  [{} {} -> {}  raw={}]\r\n",
                        action,
                        name,
                        old_pos.trim_end(),
                        new_pos.trim_end(),
                        channels[idx],
                    );
                }
                uart.write(line.as_bytes()).await.ok();
                last_switch[idx] = new_pos;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn process_bytes(
    bytes: &[u8],
    parser: &mut CrsfParser,
    channels: &mut [u16; 16],
    link_quality: &mut u8,
    have_channels: &mut bool,
    last_packet: &mut Option<Instant>,
    frame_count: &mut u32,
    parse_err: &mut u32,
    led_blue: &mut Output<'static>,
) {
    let mut remaining: &[u8] = bytes;
    while !remaining.is_empty() {
        let (result, rest) = parser.push_bytes(remaining);
        remaining = rest;
        match result {
            Some(Ok(raw)) => match raw.to_packet() {
                Ok(Packet::RcChannelsPacked(pkt)) => {
                    *channels = pkt.0;
                    *have_channels = true;
                    *last_packet = Some(Instant::now());
                    *frame_count = frame_count.wrapping_add(1);
                    led_blue.toggle();
                }
                Ok(Packet::LinkStatistics(stats)) => {
                    *link_quality = stats.uplink_link_quality;
                }
                Ok(_) => {}
                Err(_) => {
                    *parse_err = parse_err.saturating_add(1);
                    parser.reset();
                }
            },
            Some(Err(_)) => {
                *parse_err = parse_err.saturating_add(1);
                parser.reset();
            }
            None => break,
        }
    }
}

