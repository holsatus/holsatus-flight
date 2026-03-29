//! Lightweight diagnostic log channel.
//!
//! Any task (including interrupt-executor tasks) can call `log()` or `logf()`
//! to push a message into the channel without blocking. A thread-level
//! uart_writer_task in the flight binary drains the channel and writes to
//! USART1 via DMA UART.
//!
//! `log(s)` copies a `&str` (including `&'static str`) into the channel.
//! `logf(s)` moves a pre-built `heapless::String<LOG_LEN>` into the channel.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use heapless::String;

/// Maximum length of a single log message.
pub const LOG_LEN: usize = 64;

/// Capacity sized for a burst of startup messages from multiple tasks.
pub static CHANNEL: Channel<CriticalSectionRawMutex, String<LOG_LEN>, 32> = Channel::new();

/// Send a `&str` to the log channel. Truncated to `LOG_LEN` if too long.
/// Drops the message silently if the channel is full.
pub fn log(msg: &str) {
    let mut s: String<LOG_LEN> = String::new();
    // push_str returns Err on truncation; we accept truncation silently
    let _ = s.push_str(msg);
    CHANNEL.try_send(s).ok();
}
