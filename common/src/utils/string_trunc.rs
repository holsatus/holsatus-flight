//! Truncate a `&str` to at most `cap` bytes at a UTF-8 char boundary.
//!
//! Exists because `heapless::String::push_str` is *atomic* on overflow:
//! when the source doesn't fit it returns `Err` and leaves the destination
//! unchanged. Code that builds into a fresh `String<N>` and silently
//! ignores the `Err` ends up enqueuing an empty string. The H743v2 log
//! pipeline hit exactly this in 2026-05: an over-long Manual-mode log
//! line yielded an empty buffer that later crashed `uart_writer_task`
//! inside `embassy-stm32`'s DMA `Channel::configure` (assertion
//! `mem_len > 0 && mem_len <= 0xFFFF`).
//!
//! Always use this helper before pushing into a smaller `heapless::String`.

/// Largest prefix of `s` that is at most `cap` bytes long and ends on a
/// UTF-8 char boundary. Returns `""` only when `s` is empty or every
/// non-empty prefix would split a multi-byte char — both of which the
/// caller should treat as "skip the message".
pub fn truncate_to_byte_cap(s: &str, cap: usize) -> &str {
    if s.is_empty() {
        return "";
    }
    let mut end = s.len().min(cap);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    &s[..end]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_returns_empty() {
        assert_eq!(truncate_to_byte_cap("", 64), "");
        assert_eq!(truncate_to_byte_cap("", 0), "");
    }

    #[test]
    fn fits_unchanged() {
        assert_eq!(truncate_to_byte_cap("hello", 64), "hello");
        assert_eq!(truncate_to_byte_cap("hello", 5), "hello");
    }

    #[test]
    fn truncates_ascii_at_cap() {
        let s = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdefXXX";
        let out = truncate_to_byte_cap(s, 64);
        assert_eq!(out.len(), 64);
        assert_eq!(out, &s[..64]);
    }

    #[test]
    fn cap_zero_returns_empty() {
        // The DMA driver will reject empty buffers, so the caller should
        // skip the send when this happens. Documented at the call site.
        assert_eq!(truncate_to_byte_cap("hello", 0), "");
    }

    #[test]
    fn never_splits_utf8_multibyte() {
        // "é" is 2 bytes (0xC3 0xA9); cap=1 would split it.
        assert_eq!(truncate_to_byte_cap("é", 1), "");
        assert_eq!(truncate_to_byte_cap("aé", 2), "a");
        assert_eq!(truncate_to_byte_cap("aé", 3), "aé");
        // 3-byte char (Devanagari "ह" = 0xE0 0xA4 0xB9).
        assert_eq!(truncate_to_byte_cap("aहb", 2), "a");
        assert_eq!(truncate_to_byte_cap("aहb", 3), "a");
        assert_eq!(truncate_to_byte_cap("aहb", 4), "aह");
    }

    #[test]
    fn manual_log_line_regression() {
        // The exact line that crashed free_test on 2026-05-08 (length ~85).
        // Under the old log() behaviour this got pushed into String<64>,
        // push_str failed atomically, and an empty string was enqueued.
        // The truncation helper must emit a non-empty prefix instead.
        let line = "[manual] r=+0.00 p=+0.00 y=+0.00 thr=0.00 | rate r=+0.00 p=+0.00 y=+0.00 thrust=0.0 armed=0";
        assert!(line.len() > 64);
        let out = truncate_to_byte_cap(line, 64);
        assert!(!out.is_empty());
        assert!(out.len() <= 64);
        assert!(line.starts_with(out));
    }
}
