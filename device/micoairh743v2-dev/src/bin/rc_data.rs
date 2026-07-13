//! MicoAir H743v2 -- ELRS / CRSF receiver sniffer.
//!
//! Reads CRSF frames from a receiver (e.g. RadioMaster RP3) wired to
//! USART6 RX (PC7), decodes the 16 channels and LinkStatistics, and
//! prints a live, human-readable view to USART1 TX (PA9) for miniterm.
//!
//! **This binary cannot arm the drone.** It does not import
//! `COMMAD_ARM_VEHICLE`, does not spawn the motor governor, and does
//! not drive any DShot output. Safe to run on a fully-assembled drone
//! with battery and props attached -- motors will not spin.
//!
//! Run miniterm on the USB-serial adapter connected to PA9/GND at 115200
//! to watch the stream. No TX flashing required -- the TX15 already
//! speaks CRSF via its internal ELRS module.
//!
//! # What gets logged
//!   1. Switch transitions on CH5..CH10 with FSM-role labels:
//!        CH5=SE/KILL  CH6=SA/MODE  CH7=SB/SELECT  CH8=SC/reserved
//!        CH9=SD/TRIGGER  CH10=SF/RESTART
//!   2. Stick safe-zone crossings (matches FSM pre-flight gate
//!      thresholds: sticks within 10% of center, throttle within 2%
//!      of bottom).
//!   3. Stick gimbal movement on CH1..CH4 with a 50-LSB hysteresis,
//!      so a slow sweep prints intermediate values without flooding.
//!   4. Generic per-channel changes on CH11..CH16 with the same
//!      50-LSB hysteresis. CRSF only carries what EdgeTX mixes onto
//!      these channels, so:
//!        - On the TX15, in Model Setup -> Mixes, assign the inputs
//!          you want to verify (e.g. CH11=Trim_T1, CH12=Trim_T2,
//!          CH13=S1, CH14=S2, CH15=Func1, CH16=Func2, etc.).
//!        - Power-cycle the radio after editing mixes.
//!        - This binary will then log changes on those channels with
//!          their raw 11-bit values, so any TX15 control surface
//!          (T1-T4 trims, 1-6 number buttons, S1/S2 pots) can be
//!          verified without firmware changes.
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
use embassy_stm32::gpio::{Level, Output, Speed};
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
// Aligned to the TX15 default EdgeTX mix observed on this radio:
//   SE -> CH5  (idx 4) -> KILL                 (2-state button)
//   SA -> CH6  (idx 5) -> MODE                 (3-pos: Idle/Manual/Auto)
//   SB -> CH7  (idx 6) -> SELECT               (3-pos: Takeoff/Hover/Land)
//   SC -> CH8  (idx 7) -> reserved             (3-pos, free for future use)
//   SD -> CH9  (idx 8) -> TRIGGER              (3-pos, HIGH = fire edge)
//   SF -> CH10 (idx 9) -> RESTART              (2-state button; operator
//                                               must add EdgeTX mix
//                                               CH10=SF; until then this
//                                               channel reads a constant
//                                               default value)
// The "kind" field selects how the position string is decoded:
//   Kind::Generic - 3-pos LOW/MID/HIGH from CRSF thresholds
//   Kind::Mode    - 3-pos Idle/Manual/Auto
//   Kind::Select  - 3-pos Takeoff/Hover/Land
//   Kind::Button  - 2-state: tracks raw u16 with 100-LSB hysteresis,
//                   prints "released/pressed" + raw value. Used for
//                   SE and SF whose CRSF output may not cross the
//                   3-pos thresholds (limited-range button mixes).
#[derive(Copy, Clone)]
enum Kind {
    Generic,
    Mode,
    Select,
    Button,
}

const WATCHED: &[(usize, &str, &str, Kind)] = &[
    (4, "SE", "KILL",       Kind::Button),
    (5, "SA", "MODE",       Kind::Mode),
    (6, "SB", "SELECT",     Kind::Select),
    (7, "SC", "(reserved)", Kind::Generic),
    (8, "SD", "TRIGGER",    Kind::Generic),
    (9, "SF", "RESTART",    Kind::Button),
];

const BUTTON_HYST_LSB: i32 = 100;

// Stick channel indices (CRSF AETR convention).
const ROLL_IDX: usize = 0;     // CH1
const PITCH_IDX: usize = 1;    // CH2
const THROTTLE_IDX: usize = 2; // CH3
const YAW_IDX: usize = 3;      // CH4

// CRSF channel calibration (Betaflight / ELRS convention, 11-bit values).
const CRSF_LOW: i32 = 172;    // -100%
const CRSF_MID: i32 = 992;    //    0%
const CRSF_HIGH: i32 = 1811;  // +100%

// USART6 + DMA2_STREAM2 are bound at lib level in micoairh743v2::resources::RcIrqs.
use micoairh743v2::resources::{RcIrqs, UartLogIrqs};

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

/// Render a 3-pos switch position with its FSM-role label, so the
/// operator can verify the EdgeTX mix matches firmware semantics at a
/// glance. Not used for Kind::Button (which has its own raw-value
/// reporter).
fn switch_label(kind: Kind, pos: &'static str) -> &'static str {
    match (kind, pos.trim_end()) {
        (Kind::Mode, "LOW") => "LOW (Idle)",
        (Kind::Mode, "MID") => "MID (Manual)",
        (Kind::Mode, "HIGH") => "HIGH (Auto)",
        (Kind::Select, "LOW") => "LOW (Takeoff)",
        (Kind::Select, "MID") => "MID (Hover)",
        (Kind::Select, "HIGH") => "HIGH (Land)",
        (_, "LOW") => "LOW",
        (_, "MID") => "MID",
        (_, "HIGH") => "HIGH",
        _ => pos,
    }
}

/// Convert a stick channel to a normalised [-1, 1] value with a small
/// deadband. Used here for "did the stick leave / re-enter the safe boot
/// zone" reporting; matches the FSM's pre-flight gate (stick_norm).
fn stick_norm(raw: u16) -> f32 {
    let v = raw as i32 - CRSF_MID;
    let span = (CRSF_HIGH - CRSF_LOW) as f32 / 2.0;
    (v as f32 / span).clamp(-1.0, 1.0)
}

fn stick_throttle(raw: u16) -> f32 {
    let v = (raw as i32 - CRSF_LOW) as f32 / (CRSF_HIGH - CRSF_LOW) as f32;
    v.clamp(0.0, 1.0)
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
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, dbg_cfg).unwrap();
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

    // Per-channel last-seen state for the WATCHED list.
    // 3-pos switches: last_switch[idx] is the LOW/MID/HIGH string;
    //   "" means "unknown yet" so the first packet logs an init line.
    // Buttons: last_button_raw[idx] is the last raw u16 we *reported*;
    //   u16::MAX means "unknown yet". Reported again only after the
    //   raw value changes by >= BUTTON_HYST_LSB from this anchor.
    let mut last_switch: [&'static str; 16] = [""; 16];
    let mut last_button_raw: [u16; 16] = [u16::MAX; 16];
    // Baseline raw value at link-up, used to label button transitions
    // as "pressed" or "released" relative to the at-power-up state.
    // Set on the first packet, never reset.
    let mut button_baseline: [Option<u16>; 16] = [None; 16];

    // Stick safe-zone tracking. The operator's FSM pre-flight gate
    // requires sticks centered (within 10%) and throttle at the bottom
    // (within 2%). Match those thresholds here so this binary doubles
    // as a "is the boot config safe?" verifier. Each tuple is
    // (have_seen_first_packet, was_in_safe_zone_at_last_eval).
    let mut stick_state_init = false;
    let mut last_safe_roll = false;
    let mut last_safe_pitch = false;
    let mut last_safe_yaw = false;
    let mut last_safe_thr = false;
    const STICK_CENTER_TOL: f32 = 0.10;
    const THROTTLE_BOTTOM_TOL: f32 = 0.02;

    // Hysteresis-based per-channel value tracker. Records the last raw
    // value reported for each channel and emits a new line when the
    // channel moves more than HYST_LSB from that anchor. The anchor is
    // updated on each emit so the next print fires only after another
    // HYST step. Avoids flooding miniterm during a slow sweep while
    // still capturing the full range. Initial state is u16::MAX which
    // forces a first-packet emit.
    let mut last_anchor: [u16; 16] = [u16::MAX; 16];
    const HYST_LSB: i32 = 50; // ~3% of full 11-bit CRSF span

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

        // Event-driven output: print transitions on watched channels.
        // Buttons (Kind::Button) use raw-u16 hysteresis tracking and
        // print "pressed"/"released" relative to the at-link-up
        // baseline -- robust to button mixes that don't cross the
        // 3-pos LOW/MID/HIGH thresholds. 3-pos switches use the
        // existing LOW/MID/HIGH decoder with FSM-role labels.
        if have_channels {
            for &(idx, name, action, kind) in WATCHED {
                let raw = channels[idx];
                match kind {
                    Kind::Button => {
                        // Initialise baseline on first packet.
                        let baseline = match button_baseline[idx] {
                            Some(b) => b,
                            None => {
                                button_baseline[idx] = Some(raw);
                                last_button_raw[idx] = raw;
                                let mut line: String<192> = String::new();
                                let _ = write!(
                                    line,
                                    "{} ({}) init  raw={} (baseline = released)\r\n",
                                    name, action, raw,
                                );
                                uart.write(line.as_bytes()).await.ok();
                                continue;
                            }
                        };
                        // Hysteresis from the last *reported* value, not
                        // baseline, so a slow drift doesn't spam.
                        let anchor = last_button_raw[idx] as i32;
                        if (raw as i32 - anchor).abs() < BUTTON_HYST_LSB {
                            continue;
                        }
                        last_button_raw[idx] = raw;
                        let pressed =
                            (raw as i32 - baseline as i32).abs() >= BUTTON_HYST_LSB;
                        let mut line: String<192> = String::new();
                        let _ = write!(
                            line,
                            ">>> {} <<<  [{} {}  raw={} (baseline={})]\r\n",
                            action,
                            name,
                            if pressed { "PRESSED" } else { "released" },
                            raw,
                            baseline,
                        );
                        uart.write(line.as_bytes()).await.ok();
                    }
                    Kind::Generic | Kind::Mode | Kind::Select => {
                        let new_pos = switch_pos(raw);
                        let old_pos = last_switch[idx];
                        if old_pos == new_pos {
                            continue;
                        }
                        let new_label = switch_label(kind, new_pos);
                        let mut line: String<192> = String::new();
                        if old_pos.is_empty() {
                            let _ = write!(
                                line,
                                "{} ({}) init -> {} (raw={})\r\n",
                                name, action, new_label, raw,
                            );
                        } else {
                            let old_label = switch_label(kind, old_pos);
                            let _ = write!(
                                line,
                                ">>> {} <<<  [{} {} -> {}  raw={}]\r\n",
                                action, name, old_label, new_label, raw,
                            );
                        }
                        uart.write(line.as_bytes()).await.ok();
                        last_switch[idx] = new_pos;
                    }
                }
            }

            // Stick safe-zone reporting. Logs an event whenever a stick
            // leaves the FSM pre-flight safe zone or returns to it. This
            // gives the operator a live view of whether the drone would
            // pass the boot RC sanity gate at the current control state.
            let roll_n = stick_norm(channels[ROLL_IDX]);
            let pitch_n = stick_norm(channels[PITCH_IDX]);
            let yaw_n = stick_norm(channels[YAW_IDX]);
            let thr = stick_throttle(channels[THROTTLE_IDX]);
            let safe_roll = roll_n.abs() < STICK_CENTER_TOL;
            let safe_pitch = pitch_n.abs() < STICK_CENTER_TOL;
            let safe_yaw = yaw_n.abs() < STICK_CENTER_TOL;
            let safe_thr = thr <= THROTTLE_BOTTOM_TOL;

            if !stick_state_init {
                stick_state_init = true;
                last_safe_roll = safe_roll;
                last_safe_pitch = safe_pitch;
                last_safe_yaw = safe_yaw;
                last_safe_thr = safe_thr;
                let mut line: String<192> = String::new();
                let _ = write!(
                    line,
                    "STICKS init: roll={} pitch={} yaw={} thr={} ({:.2},{:.2},{:.2},{:.2})\r\n",
                    if safe_roll { "ok" } else { "BAD" },
                    if safe_pitch { "ok" } else { "BAD" },
                    if safe_yaw { "ok" } else { "BAD" },
                    if safe_thr { "ok" } else { "BAD" },
                    roll_n, pitch_n, yaw_n, thr,
                );
                uart.write(line.as_bytes()).await.ok();
            } else {
                let report = |name: &str, was: bool, now: bool, val: f32| -> Option<String<128>> {
                    if was == now { return None; }
                    let mut s: String<128> = String::new();
                    let _ = if now {
                        write!(s, "STICK {} -> safe ({:.2})\r\n", name, val)
                    } else {
                        write!(s, "STICK {} -> OUT OF SAFE ZONE ({:.2})\r\n", name, val)
                    };
                    Some(s)
                };
                if let Some(s) = report("roll", last_safe_roll, safe_roll, roll_n) {
                    uart.write(s.as_bytes()).await.ok();
                }
                if let Some(s) = report("pitch", last_safe_pitch, safe_pitch, pitch_n) {
                    uart.write(s.as_bytes()).await.ok();
                }
                if let Some(s) = report("yaw", last_safe_yaw, safe_yaw, yaw_n) {
                    uart.write(s.as_bytes()).await.ok();
                }
                if let Some(s) = report("thr", last_safe_thr, safe_thr, thr) {
                    uart.write(s.as_bytes()).await.ok();
                }
                last_safe_roll = safe_roll;
                last_safe_pitch = safe_pitch;
                last_safe_yaw = safe_yaw;
                last_safe_thr = safe_thr;
            }

            // Gimbal movement reporter (CH1..CH4). Emits with normalised
            // [-1,1] / [0,1] values for sticks so the operator can see
            // full-range sweeps.
            for &(idx, name) in &[
                (ROLL_IDX, "ROLL"),
                (PITCH_IDX, "PITCH"),
                (YAW_IDX, "YAW"),
                (THROTTLE_IDX, "THR"),
            ] {
                let raw = channels[idx] as i32;
                let anchor = last_anchor[idx] as i32;
                let moved = last_anchor[idx] == u16::MAX
                    || (raw - anchor).abs() >= HYST_LSB;
                if moved {
                    last_anchor[idx] = channels[idx];
                    let mut line: String<128> = String::new();
                    if idx == THROTTLE_IDX {
                        let _ = write!(
                            line,
                            "STICK {:5} raw={:4} val={:.2}\r\n",
                            name, channels[idx], stick_throttle(channels[idx]),
                        );
                    } else {
                        let _ = write!(
                            line,
                            "STICK {:5} raw={:4} val={:+.2}\r\n",
                            name, channels[idx], stick_norm(channels[idx]),
                        );
                    }
                    uart.write(line.as_bytes()).await.ok();
                }
            }

            // Generic per-channel reporter for CH11..CH16 (idx 10..15).
            // CH1..CH4 are sticks (movement reporter above); CH5..CH10
            // are the WATCHED switches (CH10=SF=RESTART). Anything
            // mixed onto CH11+ in EdgeTX (T1-T4 trims, 1-6 buttons,
            // S1/S2 pots, additional 3-pos switches) shows up here as
            // soon as it changes.
            for idx in 10..16 {
                let raw = channels[idx] as i32;
                let anchor = last_anchor[idx] as i32;
                let moved = last_anchor[idx] == u16::MAX
                    || (raw - anchor).abs() >= HYST_LSB;
                if moved {
                    last_anchor[idx] = channels[idx];
                    let mut line: String<128> = String::new();
                    let _ = write!(
                        line,
                        "AUX CH{:<2} raw={:4} pos={}\r\n",
                        idx + 1,
                        channels[idx],
                        switch_pos(channels[idx]).trim_end(),
                    );
                    uart.write(line.as_bytes()).await.ok();
                }
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

