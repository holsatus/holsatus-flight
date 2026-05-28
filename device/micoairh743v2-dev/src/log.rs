//! Lightweight diagnostic log channel.
//!
//! Any task (including interrupt-executor tasks) can call `log()` or `logf()`
//! to push a message into the channel without blocking. A thread-level
//! uart_writer_task in the flight binary drains the channel and writes to
//! USART1 via DMA UART.
//!
//! `log(s)` copies a `&str` (including `&'static str`) into the channel.
//! `logf(s)` moves a pre-built `heapless::String<LOG_LEN>` into the channel.

use core::sync::atomic::AtomicU8;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use heapless::String;

/// Maximum length of a single log message. Longer inputs are truncated at
/// a UTF-8 char boundary by `make_log_string`. Sized to fit the widest
/// per-tick line (the Manual ACRO `[manual] r=... thrust=... armed=...`
/// line at ~85 chars) with slack. RAM cost: CHANNEL = 256 * (LOG_LEN + ~4).
pub const LOG_LEN: usize = 96;

/// Capacity must absorb imu_monitor's ~200 Hz burst (A+B CSV lines) while the
/// uart_writer task drains through SD writes. 256 gives roughly 1.3 s of
/// headroom, enough to ride through arming bursts and SD flushes.
pub static CHANNEL: Channel<CriticalSectionRawMutex, String<LOG_LEN>, 256> = Channel::new();

/// Dedicated priority channel for safety-critical announcements that must
/// bypass high-rate telemetry queue pressure. Sized to 4, used only by
/// rc_kill_task to announce SE kill and SF restart. uart_writer_task drains
/// this channel with priority over the regular CHANNEL so the operator
/// always sees kill/restart confirmation on miniterm, even while imu_monitor
/// is flooding the regular channel at 100 Hz.
pub static CRITICAL_CHANNEL: Channel<CriticalSectionRawMutex, String<LOG_LEN>, 4> = Channel::new();

/// Mirror channel drained by `phoenix_telem::tx_loop` and re-emitted as
/// MAVLink STATUSTEXT to the Pi companion. Every regular ulog line that
/// passes `is_telem_bridged` is also pushed here so the bench operator
/// can see FSM / sensor / arming gate messages on the mavlog web pane
/// without an SD card pull. Bounded to 16: phoenix_telem drains it every
/// 10 ms, so the only way to overflow is a true flood (in which case
/// dropping a mirror line is benign -- the original is still on UART /
/// SD).
pub static TELEM_CHANNEL: Channel<CriticalSectionRawMutex, String<LOG_LEN>, 16> = Channel::new();

/// Tri-state SD-card mount status, set by `uart_writer_task`:
///   255 = check not yet complete (startup)
///     1 = SD mounted, file writable
///     0 = SD absent / filesystem error -- motors must not arm.
///
/// The flight + sub-hover binaries block arming until this is `1`, and if it
/// is `0` they enter a distinct LED pattern instead of continuing. A missing
/// card means no telemetry -- flying without learning costs props.
pub static SD_MOUNTED: AtomicU8 = AtomicU8::new(255);

/// Bitmask of sensor-init-complete flags, set by sensor-init code:
///   bit 0 = compass (QMC5883L) init OK     (I2C2)
///   bit 1 = baro DPS310 init OK            (I2C2)
///   bit 2 = bmi270 init OK                 (SPI3)
///
/// Used by `rc_kill_task` to gate USART6 init: starting the CRSF DMA
/// stream while ANY of these inits are still running wedges them via
/// AHB-matrix bus contention. D000156/158 verified the I2C2 piece
/// (compass + DPS310). D000164 then halted at t~8.4s mid-bmi270-init
/// because the original 2-bit gate cleared as soon as the I2C inits
/// finished, but bmi270's 8 KB SPI3 config-blob upload was still
/// running -- USART6 RX DMA came up and the SPI3 transaction never
/// completed (no [bmi270] init OK line ever appeared).
///
/// All three bits must be set before rc_kill is free to bring USART6
/// up. A 5-second hard timeout in rc_kill is the safety fallback so a
/// stuck sensor doesn't deadlock the kill switch.
pub static SENSORS_READY: AtomicU8 = AtomicU8::new(0);
pub const SENSOR_COMPASS_BIT: u8 = 1 << 0;
pub const SENSOR_BARO_BIT: u8 = 1 << 1;
pub const SENSOR_BMI270_BIT: u8 = 1 << 2;
pub const SENSORS_READY_ALL: u8 =
    SENSOR_COMPASS_BIT | SENSOR_BARO_BIT | SENSOR_BMI270_BIT;

/// Truncate `msg` to at most `LOG_LEN` bytes at a UTF-8 char boundary and
/// copy into a fresh `String<LOG_LEN>`. Returns `None` for empty input so
/// the caller can skip the channel send (the embassy-stm32 DMA driver
/// asserts `mem_len > 0` and panics on empty writes).
///
/// Truncation lives in `common::utils::string_trunc::truncate_to_byte_cap`
/// so it can be host-tested. See that module for the regression history.
fn make_log_string(msg: &str) -> Option<String<LOG_LEN>> {
    let trimmed = common::utils::string_trunc::truncate_to_byte_cap(msg, LOG_LEN);
    if trimmed.is_empty() {
        return None;
    }
    let mut s: String<LOG_LEN> = String::new();
    let _ = s.push_str(trimmed);
    Some(s)
}

/// Send a `&str` to the log channel. Truncated to `LOG_LEN` if too long.
/// Drops the message silently if the channel is full.
pub fn log(msg: &str) {
    if let Some(s) = make_log_string(msg) {
        mirror_to_telem(msg, &s);
        CHANNEL.try_send(s).ok();
    }
}

/// Send a `&str` to the log channel, awaiting if the channel is full.
/// Use for critical messages that must not be dropped under load (e.g. the
/// kill / restart announcements before a software reset).
pub async fn log_reliable(msg: &str) {
    if let Some(s) = make_log_string(msg) {
        mirror_to_telem(msg, &s);
        CHANNEL.send(s).await;
    }
}

/// Send a `&str` to the priority CRITICAL_CHANNEL. Used by rc_kill_task for
/// SE kill and SF restart announcements. Small bounded queue (4 slots) with
/// a single producer -- effectively never blocks, and uart_writer_task
/// drains it with priority over the regular CHANNEL so the message is
/// printed immediately even while the regular channel is saturated.
pub async fn log_critical(msg: &str) {
    if let Some(s) = make_log_string(msg) {
        mirror_to_telem(msg, &s);
        CRITICAL_CHANNEL.send(s).await;
    }
}

/// Push a copy of the line into `TELEM_CHANNEL` when the caller's prefix
/// is in the bridged set. Non-blocking: a full mavlink mirror queue is
/// dropped silently rather than back-pressuring the ulog caller (which
/// may be a critical section or IRQ-context task).
fn mirror_to_telem(raw: &str, s: &String<LOG_LEN>) {
    if !is_telem_bridged(raw) {
        return;
    }
    TELEM_CHANNEL.try_send(s.clone()).ok();
}

/// True if the line should also be relayed to the Pi as STATUSTEXT.
/// Excludes the imu_monitor CSV firehose (`A,...` / `B,...`) and the
/// per-tick `[manual]` ACRO log; everything else (FSM, sensor init,
/// `[phoenix]`, etc.) is bridged.
pub fn is_telem_bridged(msg: &str) -> bool {
    if is_high_rate_telemetry(msg) {
        return false;
    }
    if msg.starts_with("[manual]") {
        return false;
    }
    true
}

/// Waits until BOTH channels have drained to at most `target` pending
/// messages, or until `timeout_ms` elapses. Use before a software reset so
/// that queued critical + regular log messages have a chance to reach UART /
/// SD before the MCU restarts.
pub async fn wait_for_drain(target: usize, timeout_ms: u64) {
    let start = embassy_time::Instant::now();
    loop {
        if CRITICAL_CHANNEL.len() == 0 && CHANNEL.len() <= target {
            return;
        }
        if start.elapsed().as_millis() >= timeout_ms {
            return;
        }
        embassy_time::Timer::after_millis(20).await;
    }
}

/// Returns true for high-rate CSV telemetry lines that should be written to
/// the SD card but suppressed on the debug UART so miniterm stays readable.
///
/// The convention used by the imu_monitor task is one-letter channel prefix
/// followed by a comma ("A,..." IMU+motors, "B,..." PID internals). Anything
/// else is human status output that should echo to miniterm.
pub fn is_high_rate_telemetry(msg: &str) -> bool {
    let b = msg.as_bytes();
    b.len() >= 2 && b[1] == b',' && matches!(b[0], b'A' | b'B')
}
