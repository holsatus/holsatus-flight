use serde::{Deserialize, Serialize};
use thiserror::Error;

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedIoError {
    #[error("Unspecified error kind.")]
    Other,
    #[error("An entity was not found, often a file.")]
    NotFound,
    #[error("The operation lacked the necessary privileges to complete.")]
    PermissionDenied,
    #[error("The connection was refused by the remote server.")]
    ConnectionRefused,
    #[error("The connection was reset by the remote server.")]
    ConnectionReset,
    #[error("The connection was aborted (terminated) by the remote server.")]
    ConnectionAborted,
    #[error("The network operation failed because it was not connected yet.")]
    NotConnected,
    #[error("A socket address could not be bound because the address is already in use.")]
    AddrInUse,
    #[error("A nonexistent interface was requested or the requested address was not local.")]
    AddrNotAvailable,
    #[error("The operation failed because a pipe was closed.")]
    BrokenPipe,
    #[error("An entity already exists, often a file.")]
    AlreadyExists,
    #[error("A parameter was incorrect.")]
    InvalidInput,
    #[error("Data not valid for the operation were encountered.")]
    InvalidData,
    #[error("The I/O operation's timeout expired, causing it to be canceled.")]
    TimedOut,
    #[error("This operation was interrupted.")]
    Interrupted,
    #[error("This operation is unsupported on this platform.")]
    Unsupported,
    #[error("An operation could not be completed, because it failed to allocate enough memory.")]
    OutOfMemory,
    #[error("An attempted write could not write any data.")]
    WriteZero,
    #[error("An attempted write could not write any data.")]
    UnexpectedEof,
}

impl From<embedded_io::ErrorKind> for EmbeddedIoError {
    fn from(value: embedded_io::ErrorKind) -> Self {
        use embedded_io::ErrorKind as E;
        match value {
            E::Other => Self::Other,
            E::NotFound => Self::NotFound,
            E::PermissionDenied => Self::PermissionDenied,
            E::ConnectionRefused => Self::ConnectionRefused,
            E::ConnectionReset => Self::ConnectionReset,
            E::ConnectionAborted => Self::ConnectionAborted,
            E::NotConnected => Self::NotConnected,
            E::AddrInUse => Self::AddrInUse,
            E::AddrNotAvailable => Self::AddrNotAvailable,
            E::BrokenPipe => Self::BrokenPipe,
            E::AlreadyExists => Self::AlreadyExists,
            E::InvalidInput => Self::InvalidInput,
            E::InvalidData => Self::InvalidData,
            E::TimedOut => Self::TimedOut,
            E::Interrupted => Self::Interrupted,
            E::Unsupported => Self::Unsupported,
            E::OutOfMemory => Self::OutOfMemory,
            E::WriteZero => Self::WriteZero,
            _ => Self::Other,
        }
    }
}

impl embedded_io::Error for EmbeddedIoError {
    fn kind(&self) -> embedded_io::ErrorKind {
        use embedded_io::ErrorKind as E;
        match self {
            Self::Other => E::Other,
            Self::NotFound => E::NotFound,
            Self::PermissionDenied => E::PermissionDenied,
            Self::ConnectionRefused => E::ConnectionRefused,
            Self::ConnectionReset => E::ConnectionReset,
            Self::ConnectionAborted => E::ConnectionAborted,
            Self::NotConnected => E::NotConnected,
            Self::AddrInUse => E::AddrInUse,
            Self::AddrNotAvailable => E::AddrNotAvailable,
            Self::BrokenPipe => E::BrokenPipe,
            Self::AlreadyExists => E::AlreadyExists,
            Self::InvalidInput => E::InvalidInput,
            Self::InvalidData => E::InvalidData,
            Self::TimedOut => E::TimedOut,
            Self::Interrupted => E::Interrupted,
            Self::Unsupported => E::Unsupported,
            Self::OutOfMemory => E::OutOfMemory,
            Self::WriteZero => E::WriteZero,
            _ => E::Other,
        }
    }
}

pub fn map_ree<E: embedded_io::Error>(error: embedded_io::ReadExactError<E>) -> EmbeddedIoError {
    match error {
        embedded_io::ReadExactError::UnexpectedEof => EmbeddedIoError::UnexpectedEof,
        embedded_io::ReadExactError::Other(inner) => inner.kind().into(),
    }
}
