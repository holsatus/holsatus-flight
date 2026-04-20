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

/// Maximum length of a single log message.
pub const LOG_LEN: usize = 64;

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

/// Tri-state SD-card mount status, set by `uart_writer_task`:
///   255 = check not yet complete (startup)
///     1 = SD mounted, file writable
///     0 = SD absent / filesystem error -- motors must not arm.
///
/// The flight + sub-hover binaries block arming until this is `1`, and if it
/// is `0` they enter a distinct LED pattern instead of continuing. A missing
/// card means no telemetry -- flying without learning costs props.
pub static SD_MOUNTED: AtomicU8 = AtomicU8::new(255);

/// Send a `&str` to the log channel. Truncated to `LOG_LEN` if too long.
/// Drops the message silently if the channel is full.
pub fn log(msg: &str) {
    let mut s: String<LOG_LEN> = String::new();
    // push_str returns Err on truncation; we accept truncation silently
    let _ = s.push_str(msg);
    CHANNEL.try_send(s).ok();
}

/// Send a `&str` to the log channel, awaiting if the channel is full.
/// Use for critical messages that must not be dropped under load (e.g. the
/// kill / restart announcements before a software reset).
pub async fn log_reliable(msg: &str) {
    let mut s: String<LOG_LEN> = String::new();
    let _ = s.push_str(msg);
    CHANNEL.send(s).await;
}

/// Send a `&str` to the priority CRITICAL_CHANNEL. Used by rc_kill_task for
/// SE kill and SF restart announcements. Small bounded queue (4 slots) with
/// a single producer -- effectively never blocks, and uart_writer_task
/// drains it with priority over the regular CHANNEL so the message is
/// printed immediately even while the regular channel is saturated.
pub async fn log_critical(msg: &str) {
    let mut s: String<LOG_LEN> = String::new();
    let _ = s.push_str(msg);
    CRITICAL_CHANNEL.send(s).await;
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
