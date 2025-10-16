use core::sync::atomic::Ordering;

use embedded_io_async::ErrorKind;
use portable_atomic::AtomicU8;

pub(crate) struct AtomicErrorKind(AtomicU8);

impl AtomicErrorKind {
    pub const fn new() -> Self {
        AtomicErrorKind(AtomicU8::new(Self::NO_ERROR))
    }
}

impl AtomicErrorKind {
    /// Special value to indicate that no error is present
    const NO_ERROR: u8 = u8::MAX;

    pub(crate) fn set(&self, error: ErrorKind) {
        let value = match error {
            ErrorKind::Other => 0,
            ErrorKind::NotFound => 1,
            ErrorKind::PermissionDenied => 2,
            ErrorKind::ConnectionRefused => 3,
            ErrorKind::ConnectionReset => 4,
            ErrorKind::ConnectionAborted => 5,
            ErrorKind::NotConnected => 6,
            ErrorKind::AddrInUse => 7,
            ErrorKind::AddrNotAvailable => 8,
            ErrorKind::BrokenPipe => 9,
            ErrorKind::AlreadyExists => 10,
            ErrorKind::InvalidInput => 11,
            ErrorKind::InvalidData => 12,
            ErrorKind::TimedOut => 13,
            ErrorKind::Interrupted => 14,
            ErrorKind::Unsupported => 15,
            ErrorKind::OutOfMemory => 16,
            ErrorKind::WriteZero => 17,
            _ => 0,
        };

        self.0.store(value, Ordering::Relaxed);
    }

    pub(crate) fn take(&self) -> Option<ErrorKind> {
        let value = self.0.swap(Self::NO_ERROR, Ordering::Relaxed);

        let error = match value {
            Self::NO_ERROR => return None,
            0 => ErrorKind::Other,
            1 => ErrorKind::NotFound,
            2 => ErrorKind::PermissionDenied,
            3 => ErrorKind::ConnectionRefused,
            4 => ErrorKind::ConnectionReset,
            5 => ErrorKind::ConnectionAborted,
            6 => ErrorKind::NotConnected,
            7 => ErrorKind::AddrInUse,
            8 => ErrorKind::AddrNotAvailable,
            9 => ErrorKind::BrokenPipe,
            10 => ErrorKind::AlreadyExists,
            11 => ErrorKind::InvalidInput,
            12 => ErrorKind::InvalidData,
            13 => ErrorKind::TimedOut,
            14 => ErrorKind::Interrupted,
            15 => ErrorKind::Unsupported,
            16 => ErrorKind::OutOfMemory,
            17 => ErrorKind::WriteZero,
            _ => ErrorKind::Other,
        };

        Some(error)
    }
}
