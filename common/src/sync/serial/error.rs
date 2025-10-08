use core::sync::atomic::Ordering;

use embedded_io_async::ErrorKind;
use portable_atomic::AtomicU8;

pub(crate) struct AtomicError(AtomicU8);

impl AtomicError {
    pub const fn new() -> Self {
        AtomicError(AtomicU8::new(Self::NO_ERROR))
    }
}

impl AtomicError {
    const NO_ERROR: u8 = u8::MAX;

    pub(crate) fn set(&self, error: ErrorKind) {
        let value = match error {
            ErrorKind::NotFound => 0,
            ErrorKind::PermissionDenied => 1,
            ErrorKind::ConnectionRefused => 2,
            ErrorKind::ConnectionReset => 3,
            ErrorKind::ConnectionAborted => 4,
            ErrorKind::NotConnected => 5,
            ErrorKind::AddrInUse => 6,
            ErrorKind::AddrNotAvailable => 7,
            ErrorKind::BrokenPipe => 8,
            ErrorKind::AlreadyExists => 9,
            ErrorKind::InvalidInput => 10,
            ErrorKind::InvalidData => 11,
            ErrorKind::TimedOut => 12,
            ErrorKind::Interrupted => 13,
            ErrorKind::Unsupported => 14,
            ErrorKind::OutOfMemory => 15,
            ErrorKind::WriteZero => 16,
            ErrorKind::Other => 17,
            _ => 17,
        };

        self.0.store(value, Ordering::Relaxed);
    }

    pub(crate) fn take(&self) -> Option<ErrorKind> {
        // Swap in the value for no error
        let value = self.0.swap(Self::NO_ERROR, Ordering::Relaxed);

        let error = match value {
            Self::NO_ERROR => return None,
            0 => ErrorKind::NotFound,
            1 => ErrorKind::PermissionDenied,
            2 => ErrorKind::ConnectionRefused,
            3 => ErrorKind::ConnectionReset,
            4 => ErrorKind::ConnectionAborted,
            5 => ErrorKind::NotConnected,
            6 => ErrorKind::AddrInUse,
            7 => ErrorKind::AddrNotAvailable,
            8 => ErrorKind::BrokenPipe,
            9 => ErrorKind::AlreadyExists,
            10 => ErrorKind::InvalidInput,
            11 => ErrorKind::InvalidData,
            12 => ErrorKind::TimedOut,
            13 => ErrorKind::Interrupted,
            14 => ErrorKind::Unsupported,
            15 => ErrorKind::OutOfMemory,
            16 => ErrorKind::WriteZero,
            _ => ErrorKind::Other,
        };

        Some(error)
    }
}
