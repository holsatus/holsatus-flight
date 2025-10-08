use block_device_adapters::BufStreamError;
use embedded_fatfs::Error;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::errors::HolsatusError;

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedFatfsError {
    #[error("An error occured with the underlying I/O device.")]
    IoDevice,
    #[error("A read operation cannot be completed because an end of a file has been reached prematurely.")]
    UnexpectedEof,
    #[error("A write operation cannot be completed because `Write::write` returned 0.")]
    WriteZero,
    #[error("A parameter was incorrect.")]
    InvalidInput,
    #[error("A requested file or directory has not been found.")]
    NotFound,
    #[error("A file or a directory with the same name already exists.")]
    AlreadyExists,
    #[error("An operation cannot be finished because a directory is not empty.")]
    DirectoryIsNotEmpty,
    #[error("File system internal structures are corrupted/invalid.")]
    CorruptedFileSystem,
    #[error("There is not enough free space on the storage to finish the requested operation.")]
    NotEnoughSpace,
    #[error("The provided file name is either too long or empty.")]
    InvalidFileNameLength,
    #[error("The provided file name contains an invalid character.")]
    UnsupportedFileNameCharacter,
    #[error("Some other error occurred.")]
    Other,
}

impl<E> From<Error<BufStreamError<E>>> for EmbeddedFatfsError {
    fn from(value: Error<BufStreamError<E>>) -> Self {
        match value {
            Error::Io(_) => Self::IoDevice,
            Error::UnexpectedEof => Self::UnexpectedEof,
            Error::WriteZero => Self::WriteZero,
            Error::InvalidInput => Self::InvalidInput,
            Error::NotFound => Self::NotFound,
            Error::AlreadyExists => Self::AlreadyExists,
            Error::DirectoryIsNotEmpty => Self::DirectoryIsNotEmpty,
            Error::CorruptedFileSystem => Self::CorruptedFileSystem,
            Error::NotEnoughSpace => Self::NotEnoughSpace,
            Error::InvalidFileNameLength => Self::InvalidFileNameLength,
            Error::UnsupportedFileNameCharacter => Self::UnsupportedFileNameCharacter,
            _ => Self::Other,
        }
    }
}

impl<E> From<Error<BufStreamError<E>>> for HolsatusError {
    fn from(value: Error<BufStreamError<E>>) -> Self {
        EmbeddedFatfsError::from(value).into()
    }
}
