//! RC frontend: kill switch + channel publisher.
//!
//! Owns USART6 RX (PC7, 420 kbps CRSF) and is the only CRSF reader on
//! the board. Decodes RcChannelsPacked frames with a small inline
//! framer (the upstream `common::parsers::crsf::CrsfParser` halts the
//! H743v2 thread executor on production CRSF traffic from the RP3
//! receiver -- see `mission_fsm.md` and the project memory for the
//! bisection that pinpointed `push_bytes`). The framer is small and
//! self-contained: SYNC=0xC8, length 2..=62, CRC8 polynomial 0xD5
//! over [type, payload], type 0x16 = RcChannelsPacked. Other CRSF
//! frame types (LinkStatistics 0x14, telemetry, etc.) are
//! CRC-validated and dropped silently.
//!
//! TX15 EdgeTX channel map (must match Model Setup -> Mixes):
//!   CH1=Ail  CH2=Ele  CH3=Thr  CH4=Rud
//!   CH5=SE  (KILL, 2-state button)
//!   CH6=SA  (MODE: Idle / Manual / Auto)
//!   CH7=SB  (SELECT: Takeoff / Hover / Land)
//!   CH8=SC  (reserved)
//!   CH9=SD  (TRIGGER, 3-pos used as a stiff momentary; HIGH = press)
//!   CH10=SF (RESTART, 2-state button -- requires `CH10 = source SF`
//!            mix and ELRS Switch Mode = 12ch wide / 16ch wide)
//!
//! On every valid RcChannelsPacked frame the task:
//!   1. Records an SE/SF baseline on the very first frame and sets
//!      `RC_LINK_READY` so the FSM can proceed.
//!   2. Compares SE / SF raw values against baseline with a 100-LSB
//!      hysteresis (button outputs may not cross the 3-pos LOW/MID/HIGH
//!      thresholds). Any change disarms; SF then `SCB::sys_reset()`s.
//!   3. Decodes SA/SB/SD into Mode / Maneuver / TriggerPressed and
//!      publishes RC_CHANNELS + edge events on RC_EVENT.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};

use common::tasks::commander::COMMAD_ARM_VEHICLE;
use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use heapless::String;

use crate::log as ulog;
use crate::resources::{RcIrqs, RcResources};

/// True after the first valid CRSF frame establishes the SE/SF baseline.
/// Mission sequencers must wait for this before arming, so the kill
/// switch is functional before any motor spins.
pub static RC_LINK_READY: AtomicBool = AtomicBool::new(false);

/// Latest decoded RC channel state. Bumped per CRSF frame.
pub static RC_CHANNELS: Watch<CriticalSectionRawMutex, RcChannels, 4> = Watch::new();

/// Edge events: mode level changes, select level changes, trigger
/// rising edges. Bounded queue; old events drop on overflow if the FSM
/// is slow.
pub static RC_EVENT: Channel<CriticalSectionRawMutex, RcEvent, 8> = Channel::new();

pub const BAUD: u32 = 420_000;

// CRSF channel indices (0-based) on this radio's EdgeTX mix.
const SE_IDX: usize = 4;
const SA_IDX: usize = 5;
const SB_IDX: usize = 6;
const SD_IDX: usize = 8;
const SF_IDX: usize = 9;

// Stick channel indices (CRSF AETR convention).
pub const ROLL_IDX: usize = 0;
pub const PITCH_IDX: usize = 1;
pub const THROTTLE_IDX: usize = 2;
pub const YAW_IDX: usize = 3;

// CRSF 11-bit channel calibration values.
const CRSF_LOW: i32 = 172;
const CRSF_MID: i32 = 992;
const CRSF_HIGH: i32 = 1811;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Pos {
    Low,
    Mid,
    High,
}

pub fn decode(raw: u16) -> Pos {
    let v = raw as i32;
    if v < (CRSF_LOW + CRSF_MID) / 2 {
        Pos::Low
    } else if v > (CRSF_MID + CRSF_HIGH) / 2 {
        Pos::High
    } else {
        Pos::Mid
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
    pub raw: [u16; 16],
    pub seq: u32,
    pub mode: Mode,
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

/// Stick channel -> normalised [-1, 1] with 4% deadband. For roll/pitch/yaw.
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

/// Throttle channel -> [0, 1] with 5% bottom deadband.
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

// CRSF framer constants.
const CRSF_SYNC: u8 = 0xC8;
const CRSF_LEN_MIN: u8 = 2;
const CRSF_LEN_MAX: u8 = 62;
const CRSF_RC_CHANNELS_TYPE: u8 = 0x16;
const CRSF_RC_CHANNELS_TOTAL: usize = 26;
const CRSF_BUF_LEN: usize = 64;

enum FrameState {
    SeekSync,
    ReadLen,
    ReadPayload,
}

/// Minimal CRSF byte-level framer. Feed bytes one at a time via
/// [`feed_byte`]; on a validated RcChannelsPacked frame it returns
/// `Some([u16; 16])`. Other frame types are CRC-validated and silently
/// dropped (returns None).
struct CrsfFramer {
    state: FrameState,
    frame: [u8; CRSF_BUF_LEN],
    frame_idx: usize,
    expected_total: usize,
}

impl CrsfFramer {
    const fn new() -> Self {
        Self {
            state: FrameState::SeekSync,
            frame: [0u8; CRSF_BUF_LEN],
            frame_idx: 0,
            expected_total: 0,
        }
    }

    fn reset(&mut self) {
        self.state = FrameState::SeekSync;
        self.frame_idx = 0;
        self.expected_total = 0;
    }

    fn feed_byte(&mut self, byte: u8) -> Option<[u16; 16]> {
        match self.state {
            FrameState::SeekSync => {
                if byte == CRSF_SYNC {
                    self.frame[0] = byte;
                    self.frame_idx = 1;
                    self.state = FrameState::ReadLen;
                }
                None
            }
            FrameState::ReadLen => {
                if (CRSF_LEN_MIN..=CRSF_LEN_MAX).contains(&byte) {
                    self.frame[1] = byte;
                    self.frame_idx = 2;
                    self.expected_total = 2 + byte as usize;
                    self.state = FrameState::ReadPayload;
                } else {
                    self.reset();
                }
                None
            }
            FrameState::ReadPayload => {
                if self.frame_idx >= self.frame.len() {
                    self.reset();
                    return None;
                }
                self.frame[self.frame_idx] = byte;
                self.frame_idx += 1;
                if self.frame_idx < self.expected_total {
                    return None;
                }

                let result = self.try_decode();
                self.reset();
                result
            }
        }
    }

    fn try_decode(&self) -> Option<[u16; 16]> {
        // CRC8 (poly 0xD5) over [type, payload], excludes CRC byte.
        let crc_input = &self.frame[2..self.expected_total - 1];
        let received_crc = self.frame[self.expected_total - 1];
        if crc8_d5(crc_input) != received_crc {
            return None;
        }

        if self.frame[2] != CRSF_RC_CHANNELS_TYPE
            || self.expected_total != CRSF_RC_CHANNELS_TOTAL
        {
            return None;
        }

        let payload: &[u8; 22] = (&self.frame[3..25]).try_into().ok()?;
        Some(common::parsers::crsf::packet_definitions::rc_channels_packed::raw_decode(payload).0)
    }
}

fn crc8_d5(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// SE/SF buttons may not cross the 3-pos LOW/MID/HIGH thresholds when
/// the EdgeTX mix scales them tightly. Compare raw u16 values against
/// baseline with a 100-LSB hysteresis to catch any meaningful press.
const BUTTON_HYSTERESIS: i32 = 100;
fn raw_changed(prev: u16, cur: u16) -> bool {
    (cur as i32 - prev as i32).abs() >= BUTTON_HYSTERESIS
}

#[embassy_executor::task]
pub async fn rc_kill_task(r: RcResources) -> ! {
    // Wait for I2C2 (compass + DPS310) and SPI3 (bmi270) sensor inits
    // before bringing USART6 up. Concurrent USART6 RX DMA traffic
    // wedges those inits on the H743v2 via AHB-matrix bus contention.
    // 5 s timeout fallback so a stuck sensor never deadlocks the kill
    // switch -- USART6 init proceeds anyway and SE/SF still work.
    {
        const SENSOR_WAIT_TIMEOUT_MS: u64 = 5_000;
        let start = embassy_time::Instant::now();
        loop {
            let mask = ulog::SENSORS_READY.load(Ordering::Acquire);
            if mask & ulog::SENSORS_READY_ALL == ulog::SENSORS_READY_ALL {
                break;
            }
            if start.elapsed().as_millis() as u64 >= SENSOR_WAIT_TIMEOUT_MS {
                let mut s: String<96> = String::new();
                let _ = write!(
                    s,
                    "[rc_kill] sensor-ready timeout (mask=0x{:02x}) -- starting USART6 anyway",
                    mask,
                );
                ulog::log(s.as_str());
                break;
            }
            Timer::after_millis(100).await;
        }
    }

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

    ulog::log("[rc_kill] USART6@420000 ready, watching SE (CH5) and SF (CH10)");

    let snd_channels = RC_CHANNELS.sender();
    // Mirror the raw 16-channel array onto the common-crate Watch so
    // downstream code that doesn't depend on the H743v2 crate (e.g.
    // phoenix_telem RC_CHANNELS MAVLink frames -> Pi /fc/rc_input) sees
    // the same data. The H743v2 has its own rc_kill task instead of
    // common::tasks::rc_reader (the upstream parser deadlocks under live
    // CRSF traffic from the RP3, see rc_kill module docs), so the common
    // signal would otherwise stay empty forever.
    let mut snd_chans_raw = common::signals::RC_CHANNELS_RAW.sender();
    let mut framer = CrsfFramer::new();
    let mut buf = [0u8; 64];

    let mut baseline: Option<(u16, u16)> = None;
    let mut killed = false;
    let mut seq: u32 = 0;
    let mut last_mode: Option<Mode> = None;
    let mut last_maneuver: Option<Maneuver> = None;
    let mut last_trigger: Option<Pos> = None;

    loop {
        let n = match uart.read_until_idle(&mut buf).await {
            Ok(n) if n > 0 => n,
            _ => continue,
        };

        for i in 0..n {
            let Some(channels) = framer.feed_byte(buf[i]) else {
                continue;
            };

            // Publish the raw 16-channel frame on the shared Watch every
            // valid CRSF packet, before any kill/restart short-circuits.
            // Done once here rather than at every per-branch send below so
            // we can't miss a frame on an early return path.
            snd_chans_raw.send(Some(channels));

            let se_raw = channels[SE_IDX];
            let sf_raw = channels[SF_IDX];
            let mode = Mode::from_pos(decode(channels[SA_IDX]));
            let maneuver = Maneuver::from_pos(decode(channels[SB_IDX]));
            let sd_pos = decode(channels[SD_IDX]);

            // First frame: record baseline + announce link.
            let (se0, sf0) = match baseline {
                None => {
                    baseline = Some((se_raw, sf_raw));
                    snd_channels.send(RcChannels {
                        raw: channels,
                        seq: 0,
                        mode,
                        maneuver,
                    });
                    RC_LINK_READY.store(true, Ordering::Relaxed);
                    let mut s: String<96> = String::new();
                    let _ = write!(
                        s,
                        "[rc_kill] baseline SE_raw={} SF_raw={} mode={} sel={}",
                        se_raw,
                        sf_raw,
                        mode.label(),
                        maneuver.label(),
                    );
                    ulog::log(s.as_str());
                    (se_raw, sf_raw)
                }
                Some(b) => b,
            };

            // SF = RESTART. Checked first so it always works, even
            // after SE has latched a kill.
            if raw_changed(sf0, sf_raw) {
                // Latch first: a plain store cannot be blocked, so the
                // keepalive stops the motors even if everything below
                // wedges. Cleared by the reboot itself.
                crate::dshot_driver::MOTOR_KILL.store(true, Ordering::Relaxed);
                COMMAD_ARM_VEHICLE.send(false);
                let mut s: String<96> = String::new();
                let _ = write!(
                    s,
                    "[kill] RC RESTART: SF raw {} -> {} -- rebooting when drained",
                    sf0, sf_raw,
                );
                ulog::log_critical(s.as_str()).await;
                ulog::wait_for_drain(4, 3000).await;
                cortex_m::peripheral::SCB::sys_reset();
            }

            // SE = KILL. Latches disarm on first press.
            if raw_changed(se0, se_raw) && !killed {
                // Latch BEFORE the watch send and the log: the store is the
                // one operation that cannot be blocked by a wedged governor,
                // a full log channel, or a lost wake (2026-07-08 incident).
                // The keepalive forces DShot 0 from its next 1 ms frame.
                crate::dshot_driver::MOTOR_KILL.store(true, Ordering::Relaxed);
                COMMAD_ARM_VEHICLE.send(false);
                killed = true;
                let mut s: String<96> = String::new();
                let _ = write!(
                    s,
                    "[kill] RC ABORT: SE raw {} -> {} -- motors disarmed (press SF to restart)",
                    se0, se_raw,
                );
                ulog::log_critical(s.as_str()).await;
            }

            // Publish current channel state.
            seq = seq.wrapping_add(1);
            snd_channels.send(RcChannels {
                raw: channels,
                seq,
                mode,
                maneuver,
            });

            // Mode change edge.
            if last_mode != Some(mode) {
                last_mode = Some(mode);
                let _ = RC_EVENT.try_send(RcEvent::ModeChanged(mode));
                let mut s: String<48> = String::new();
                let _ = write!(s, "[rc] mode={}", mode.label());
                ulog::log(s.as_str());
            }

            // Maneuver-select change edge.
            if last_maneuver != Some(maneuver) {
                last_maneuver = Some(maneuver);
                let _ = RC_EVENT.try_send(RcEvent::ManeuverSelected(maneuver));
                let mut s: String<48> = String::new();
                let _ = write!(s, "[rc] select={}", maneuver.label());
                ulog::log(s.as_str());
            }

            // SD trigger rising edge: any -> High.
            let trig_now_high = sd_pos == Pos::High;
            let trig_was_high = matches!(last_trigger, Some(Pos::High));
            if trig_now_high && !trig_was_high {
                let _ = RC_EVENT.try_send(RcEvent::TriggerPressed);
                ulog::log("[rc] trigger");
            }
            last_trigger = Some(sd_pos);
        }
    }
}
