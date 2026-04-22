//! RC kill switch.
//!
//! Reads CRSF frames from an ExpressLRS receiver on USART6 (PC7, 420000 baud)
//! and acts on SE / SF switch transitions from the TX15:
//!
//!   SE (CH5) changes -> permanent kill (disarm + latch forever, same
//!                       semantics as flip_kill / gyro_runaway_kill)
//!   SF (CH6) changes -> disarm, then software reset of the MCU, which
//!                       restarts the whole binary from boot (IMU cal,
//!                       AHRS settle, RC-link re-acquire, re-arm)
//!
//! Baseline positions are recorded from the first RcChannelsPacked packet
//! received, so the pilot can start with either switch in any position.
//!
//! On the TX15 (EdgeTX) the corresponding mixes must exist:
//!   Model Setup -> Mixes -> CH5 source = SE, CH6 source = SF.
//!
//! If the RC link never establishes, this task simply sits waiting for bytes
//! and never triggers -- motor safety in that scenario is the responsibility
//! of flip_kill / gyro_runaway_kill.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};

use common::parsers::crsf::packet_containers::Packet;
use common::parsers::crsf::CrsfParser;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals;
use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_time::Timer;
use heapless::String;

use crate::log as ulog;
use crate::resources::RcResources;

bind_interrupts!(pub struct RcIrqs {
    USART6       => embassy_stm32::usart::InterruptHandler<peripherals::USART6>;
    DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
});

/// Set to true by the RC kill task after the first valid CRSF packet establishes
/// a baseline. Mission sequencers must wait for this before arming so that the
/// kill switch is functional before any motor spins.
pub static RC_LINK_READY: AtomicBool = AtomicBool::new(false);

pub const BAUD: u32 = 420_000;

// Channel indices (0-based) of the watched switches.
const SE_IDX: usize = 4; // CH5 = AUX1
const SF_IDX: usize = 5; // CH6 = AUX2

// CRSF 11-bit channel calibration (Betaflight / ELRS convention).
const CRSF_LOW: i32 = 172;
const CRSF_MID: i32 = 992;
const CRSF_HIGH: i32 = 1811;

#[derive(Copy, Clone, PartialEq, Eq)]
enum Pos {
    Low,
    Mid,
    High,
}

fn decode(raw: u16) -> Pos {
    let v = raw as i32;
    if v < (CRSF_LOW + CRSF_MID) / 2 {
        Pos::Low
    } else if v > (CRSF_MID + CRSF_HIGH) / 2 {
        Pos::High
    } else {
        Pos::Mid
    }
}

fn label(p: Pos) -> &'static str {
    match p {
        Pos::Low => "LOW",
        Pos::Mid => "MID",
        Pos::High => "HIGH",
    }
}

#[embassy_executor::task]
pub async fn rc_kill_task(r: RcResources) -> ! {
    let mut cfg = UartConfig::default();
    cfg.baudrate = BAUD;

    let mut uart = match UartRx::new(r.usart, r.rx, r.dma, RcIrqs, cfg) {
        Ok(u) => u,
        Err(_) => {
            ulog::log("[rc_kill] USART6 init FAILED -- task inactive");
            loop {
                Timer::after_secs(60).await;
            }
        }
    };

    ulog::log("[rc_kill] USART6@420000 ready, watching SE (CH5) and SF (CH6)");

    let mut parser = CrsfParser::new();
    let mut buf = [0u8; 64];
    let mut baseline: Option<(Pos, Pos)> = None;
    // Latched once an SE flip has killed the vehicle. Prevents repeated
    // kill-log spam on subsequent SE wiggles, but we *keep running* the
    // read loop so SF (restart) still works after SE has been pressed.
    let mut killed = false;

    loop {
        let n = match uart.read_until_idle(&mut buf).await {
            Ok(n) if n > 0 => n,
            _ => continue,
        };

        let mut remaining: &[u8] = &buf[..n];
        while !remaining.is_empty() {
            let (result, rest) = parser.push_bytes(remaining);
            remaining = rest;

            let packet = match result {
                Some(Ok(raw)) => raw.to_packet(),
                Some(Err(_)) => {
                    parser.reset();
                    continue;
                }
                None => break,
            };

            let Ok(Packet::RcChannelsPacked(pkt)) = packet else {
                continue;
            };

            let se = decode(pkt.0[SE_IDX]);
            let sf = decode(pkt.0[SF_IDX]);

            match baseline {
                None => {
                    baseline = Some((se, sf));
                    RC_LINK_READY.store(true, Ordering::Relaxed);
                    let mut s: String<96> = String::new();
                    let _ = write!(
                        s,
                        "[rc_kill] baseline SE={} SF={} -- any change triggers kill",
                        label(se),
                        label(sf),
                    );
                    ulog::log(s.as_str());
                }
                Some((se0, sf0)) => {
                    // SF = RESTART. Checked first so it *always* works,
                    // including after SE has already killed the vehicle.
                    // Disarms, lets the motor governor send disarm DShot
                    // frames and the log writer flush, then software-
                    // resets the MCU -- equivalent to a power cycle.
                    if sf != sf0 {
                        COMMAD_ARM_VEHICLE.send(false);

                        // Use log_reliable so this announcement is guaranteed
                        // to reach miniterm / SD even when the log channel is
                        // saturated by imu_monitor at ~200 Hz.
                        let mut s: String<96> = String::new();
                        let _ = write!(
                            s,
                            "[kill] RC RESTART: SF {} -> {} -- rebooting when drained",
                            label(sf0),
                            label(sf),
                        );
                        ulog::log_critical(s.as_str()).await;

                        // Wait for the channel to substantially drain before
                        // resetting, so any earlier [kill] SE message AND the
                        // restart announcement above actually reach the UART
                        // / SD writer. Capped at 3 s so we never hang.
                        ulog::wait_for_drain(4, 3000).await;

                        cortex_m::peripheral::SCB::sys_reset();
                    }

                    // SE = KILL. Latches disarm on first flip. We do NOT
                    // exit the read loop here -- that would prevent SF
                    // from being observed for the remainder of the run.
                    // Subsequent SE wiggles are ignored once `killed`.
                    if se != se0 && !killed {
                        COMMAD_ARM_VEHICLE.send(false);
                        killed = true;

                        let mut s: String<96> = String::new();
                        let _ = write!(
                            s,
                            "[kill] RC ABORT: SE {} -> {} -- motors disarmed (flip SF to restart)",
                            label(se0),
                            label(se),
                        );
                        ulog::log_critical(s.as_str()).await;
                    }
                }
            }
        }
    }
}
