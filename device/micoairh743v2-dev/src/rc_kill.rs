//! RC frontend: kill switch + channel publisher.
//!
//! Reads CRSF frames from an ExpressLRS receiver on USART6 (PC7, 420000 baud)
//! and acts on SE / SF switch transitions from the TX15. Also decodes the
//! mode / select / trigger switches and publishes channel state + edge
//! events for the mission FSM to consume.
//!
//!   SE (CH5) changes -> permanent kill (disarm + latch forever, same
//!                       semantics as flip_kill / gyro_runaway_kill)
//!   SF (CH6) changes -> disarm, then software reset of the MCU, which
//!                       restarts the whole binary from boot (IMU cal,
//!                       AHRS settle, RC-link re-acquire, re-arm)
//!   SA (CH7) level   -> MODE: Idle (Low) / Manual (Mid) / Auto (High)
//!   SB (CH8) level   -> SELECT: Takeoff (Low) / Hover (Mid) / Land (High)
//!   SH (CH9) edge    -> TRIGGER (rising edge fires selected maneuver)
//!
//! See `mission_fsm.md` for the full channel map and operating procedure.
//!
//! Baseline positions for SE/SF are recorded from the first RcChannelsPacked
//! packet received, so the pilot can start with either switch in any
//! position. Mode/select/trigger have no baseline -- they are reported in
//! absolute terms.
//!
//! On the TX15 (EdgeTX) the corresponding mixes must exist:
//!   CH5=SE, CH6=SF, CH7=SA, CH8=SB, CH9=SH.
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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
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

/// Latest decoded RC channel state. Bumped per CRSF packet. Receivers:
/// the mission FSM, optional debug loggers. Must be initialised on link
/// acquire so `try_get()` does not return None.
pub static RC_CHANNELS: Watch<CriticalSectionRawMutex, RcChannels, 4> = Watch::new();

/// Edge events derived from RC channels: mode level changes, select
/// level changes, trigger rising edges. Bounded queue so a non-running
/// FSM cannot stall the RC reader -- old events drop on overflow.
pub static RC_EVENT: Channel<CriticalSectionRawMutex, RcEvent, 8> = Channel::new();

pub const BAUD: u32 = 420_000;

// Channel indices (0-based) of the watched switches.
const SE_IDX: usize = 4; // CH5 = AUX1
const SF_IDX: usize = 5; // CH6 = AUX2
const SA_IDX: usize = 6; // CH7 = AUX3 (MODE)
const SB_IDX: usize = 7; // CH8 = AUX4 (SELECT)
const SH_IDX: usize = 8; // CH9 = AUX5 (TRIGGER)

// CRSF 11-bit channel calibration (Betaflight / ELRS convention).
const CRSF_LOW: i32 = 172;
const CRSF_MID: i32 = 992;
const CRSF_HIGH: i32 = 1811;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Pos {
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

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Mode {
    Idle,
    Manual,
    Auto,
}

impl Mode {
    fn from_pos(p: Pos) -> Self {
        match p {
            Pos::Low => Mode::Idle,
            Pos::Mid => Mode::Manual,
            Pos::High => Mode::Auto,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Mode::Idle => "Idle",
            Mode::Manual => "Manual",
            Mode::Auto => "Auto",
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Maneuver {
    Takeoff,
    Hover,
    Land,
}

impl Maneuver {
    fn from_pos(p: Pos) -> Self {
        match p {
            Pos::Low => Maneuver::Takeoff,
            Pos::Mid => Maneuver::Hover,
            Pos::High => Maneuver::Land,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Maneuver::Takeoff => "Takeoff",
            Maneuver::Hover => "Hover",
            Maneuver::Land => "Land",
        }
    }
}

#[derive(Copy, Clone)]
pub struct RcChannels {
    /// Raw 11-bit CRSF channel values, indices 0..15.
    pub raw: [u16; 16],
    /// Monotonic packet counter; bumps every CRSF frame received.
    pub seq: u32,
    /// Pre-decoded MODE level (SA / CH7).
    pub mode: Mode,
    /// Pre-decoded SELECT level (SB / CH8).
    pub maneuver: Maneuver,
}

impl Default for RcChannels {
    fn default() -> Self {
        RcChannels {
            raw: [CRSF_MID as u16; 16],
            seq: 0,
            mode: Mode::Idle,
            maneuver: Maneuver::Takeoff,
        }
    }
}

#[derive(Copy, Clone)]
pub enum RcEvent {
    ModeChanged(Mode),
    ManeuverSelected(Maneuver),
    TriggerPressed,
}

/// Convert a stick channel to a normalised [-1, 1] value with deadband
/// applied at the centre. Returns 0 inside the deadband. Used by the
/// FSM Manual state for roll/pitch/yaw sticks; throttle uses
/// `stick_throttle` instead since it ranges [0, 1].
pub fn stick_norm(raw: u16) -> f32 {
    let v = raw as i32 - CRSF_MID;
    let span = (CRSF_HIGH - CRSF_LOW) as f32 / 2.0;
    let n = (v as f32 / span).clamp(-1.0, 1.0);
    const DEADBAND: f32 = 0.04;
    if n.abs() < DEADBAND {
        0.0
    } else {
        let sign = if n > 0.0 { 1.0 } else { -1.0 };
        sign * (n.abs() - DEADBAND) / (1.0 - DEADBAND)
    }
}

/// Convert the throttle channel to [0, 1] with a small bottom deadband
/// so noise at idle does not produce thrust.
pub fn stick_throttle(raw: u16) -> f32 {
    let v = (raw as i32 - CRSF_LOW) as f32 / (CRSF_HIGH - CRSF_LOW) as f32;
    let v = v.clamp(0.0, 1.0);
    const IDLE_DEADBAND: f32 = 0.05;
    if v < IDLE_DEADBAND {
        0.0
    } else {
        (v - IDLE_DEADBAND) / (1.0 - IDLE_DEADBAND)
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

    let snd_channels = RC_CHANNELS.sender();
    let mut seq: u32 = 0;
    let mut last_mode: Option<Mode> = None;
    let mut last_maneuver: Option<Maneuver> = None;
    let mut last_trigger: Option<Pos> = None;

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
                    // Seed the channel watch before declaring link ready,
                    // so any FSM that races on RC_LINK_READY -> RC_CHANNELS
                    // sees a populated value on the first try_get().
                    snd_channels.send(RcChannels {
                        raw: pkt.0,
                        seq: 0,
                        mode: Mode::from_pos(decode(pkt.0[SA_IDX])),
                        maneuver: Maneuver::from_pos(decode(pkt.0[SB_IDX])),
                    });
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

            // Decode mode / select / trigger and publish channel state +
            // edge events to the FSM. Done unconditionally so the FSM
            // sees a stream from the moment the link is up, including
            // after a kill (so the FSM can return to GroundIdle).
            let sa_pos = decode(pkt.0[SA_IDX]);
            let sb_pos = decode(pkt.0[SB_IDX]);
            let sh_pos = decode(pkt.0[SH_IDX]);
            let mode = Mode::from_pos(sa_pos);
            let maneuver = Maneuver::from_pos(sb_pos);

            seq = seq.wrapping_add(1);
            snd_channels.send(RcChannels {
                raw: pkt.0,
                seq,
                mode,
                maneuver,
            });

            if last_mode != Some(mode) {
                last_mode = Some(mode);
                let _ = RC_EVENT.try_send(RcEvent::ModeChanged(mode));
                let mut s: String<48> = String::new();
                let _ = write!(s, "[rc] mode={}", mode.label());
                ulog::log(s.as_str());
            }
            if last_maneuver != Some(maneuver) {
                last_maneuver = Some(maneuver);
                let _ = RC_EVENT.try_send(RcEvent::ManeuverSelected(maneuver));
                let mut s: String<48> = String::new();
                let _ = write!(s, "[rc] select={}", maneuver.label());
                ulog::log(s.as_str());
            }
            // SH rising edge: any -> High. Treat Low and Mid as "released",
            // so a non-momentary 3-pos in Mid still counts as not-pressed.
            let trig_now_high = sh_pos == Pos::High;
            let trig_was_high = matches!(last_trigger, Some(Pos::High));
            if trig_now_high && !trig_was_high {
                let _ = RC_EVENT.try_send(RcEvent::TriggerPressed);
                ulog::log("[rc] trigger");
            }
            last_trigger = Some(sh_pos);
        }
    }
}
