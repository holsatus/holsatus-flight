use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod adapter;
use adapter::{
    embedded_fatfs::EmbeddedFatfsError, embedded_hal::*, embedded_io::*,
    embedded_storage::StorageError, postcard::PostcardError, sequential_storage::SequentialError,
};

#[non_exhaustive]
#[derive(serde::Serialize, serde::Deserialize, Error, Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HolsatusError {
    #[error("Embedded-HAL {0}")]
    EmbBus(#[from] DeviceError),
    #[error("Embedded-IO error: {0}")]
    EmbIo(#[from] EmbeddedIoError),
    #[error("Embedded-HAL I2C error: {0}")]
    EmbI2c(#[from] EmbeddedI2cError),
    #[error("Embedded-HAL SPI error: {0}")]
    EmbSpi(#[from] EmbeddedSpiError),
    #[error("Embedded-HAL digital error: {0}")]
    EmbDig(#[from] EmbeddedDigError),
    #[error("FAT File system error: {0}")]
    EmbFat(#[from] EmbeddedFatfsError),
    #[error("Postcard ser/de error: {0}")]
    Postcard(#[from] PostcardError),
    #[error("Parsing error: {0}")]
    Parse(#[from] ParseError),
    #[error("Blackbox error: {0}")]
    Blackbox(#[from] BlackboxError),
    #[error("Calibration error: {0}")]
    Calibration(#[from] CalibrationError),
    #[error("Mavlink error: {0}")]
    Mavlink(#[from] MavlinkError),
    #[error("Sequential storage error: {0}")]
    Sequential(#[from] SequentialError),
    #[error("Embedded storage error: {0}")]
    Storage(#[from] StorageError),
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceError {
    #[error("The device is not responding after {millis} ms.")]
    Timeout { millis: u64 },
    #[error("The device was not identified correctly.")]
    IdentificationError,
    #[error("An external interrupt tied ot this device failed.")]
    ExtInterruptError,
    #[error("I2c error: {0}")]
    I2c(#[from] EmbeddedI2cError),
    #[error("Spi error: {0}")]
    Spi(#[from] EmbeddedSpiError),
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    #[error("Parsed data is of an invalid format.")]
    InvalidData,
    #[error("No syncronization symbol could be located.")]
    NoSyncronization,
    #[error("Calculated checksum did not match the one received.")]
    InvalidChecksum,
    #[error("Too much time passed since last successful parsed message.")]
    Timeout,
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BlackboxError {
    #[error("Failed to reset block device.")]
    ResetFault,
    #[error("Failed to create file system instance.")]
    FsCreateFault,
    #[error("The buffer used to queue up log messages is full.")]
    TransmitBufferFull,
    #[error("The logger was unable to construct a message to save.")]
    MessageFormingError,
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalibrationError {
    #[error("The accelerometer is not sending data.")]
    AccMaxDropped,
    #[error("The requested accelerometer sensor ID is invalid.")]
    AccInvalidId,

    #[error("The gyroscope is not sending data.")]
    GyrMaxDropped,
    #[error("The variance of the accelerometer data is too high while calibration gyroscope.")]
    GyrHighVariance,
    #[error("The requested gyroscope sensor ID is invalid.")]
    GyrInvalidId,

    #[error("The magnetometer is not sending data.")]
    MagMaxDropped,
    #[error("The collected data did not fit the calibration model.")]
    MagBadFit,
    #[error("The requested magnetometer sensor ID is invalid.")]
    MagInvalidId,
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MavlinkError {
    #[error("The IO device experienced an unexpected end of file")]
    UnexpectedEof,
    #[error("Failed to read or write message from IO.")]
    GenericIoError,
    #[error("Failed to read or write message from IO: {0}")]
    IoError(#[from] EmbeddedIoError),
    #[error("The underlying IO device reached the end of file")]
    IoEndOfFile,
    #[error("Could not start stream, all streamers are currently in use.")]
    MaxStreamers,
    #[error("No more room for adding extra peers.")]
    MaxPeers,
    #[error("Invalid flag value for flag type.")]
    InvalidFlag,
    #[error("Invalid enum value for enum type.")]
    InvalidEnum,
    #[error("Message ID {0} is not part of a known dialect.")]
    UnknownMessageID(u32),
    #[error("Received message of an unsupported MAVLink version.")]
    UnsupportedVersion,
    #[error("Received message had an invalid checksum.")]
    ChecksumFailure,
    #[error("Received message had an invalid signature.")]
    SignatureFailure,
    #[error("Compatibility mismatch")]
    Incompatible,
    #[error("Unknown MAV_CMD value.")]
    UnknownMavCmd,
    #[error("The received message was malformed.")]
    MalformedMessage,
    #[error("Message ID reception not supported by this firmware.")]
    NoMessageHandler,
    #[error("Invalid payload length for a V1 message.")]
    InvalidV1Payload,
    #[error("Invalid payload length for a V2 message.")]
    V2PayloadIsTooLong,
    #[error("A remote procedure failed to respond.")]
    RpcFailure,
    #[error("MAVLink frame can’t be process due to a small buffer.")]
    FrameBufferIsTooSmall,
    #[error("MAVLink frame can’t be process due to a small buffer.")]
    InvalidHeader,
}

// #[cfg(feature = "mavio")]
impl From<mavio::error::Error> for MavlinkError {
    fn from(value: mavio::error::Error) -> Self {
        match value {
            mavio::error::Error::Io(io_error) => io_error.into(),
            mavio::error::Error::Frame(frame_error) => frame_error.into(),
            mavio::error::Error::Spec(spec_error) => spec_error.into(),
        }
    }
}

impl From<mavio::error::FrameError> for MavlinkError {
    fn from(value: mavio::error::FrameError) -> Self {
        match value {
            mavio::error::FrameError::Version(_) => MavlinkError::UnsupportedVersion,
            mavio::error::FrameError::Checksum => MavlinkError::ChecksumFailure,
            mavio::error::FrameError::Signature => MavlinkError::SignatureFailure,
            mavio::error::FrameError::Incompatible(_) => MavlinkError::Incompatible,
            mavio::error::FrameError::NotInDialect(id) => MavlinkError::UnknownMessageID(id),
            mavio::error::FrameError::FrameBufferIsTooSmall { .. } => {
                MavlinkError::FrameBufferIsTooSmall
            }
            mavio::error::FrameError::InvalidHeader => MavlinkError::InvalidHeader,
        }
    }
}

impl From<mavio::error::SpecError> for MavlinkError {
    fn from(value: mavio::error::SpecError) -> Self {
        match value {
            mavio::error::SpecError::UnsupportedMavLinkVersion { .. } => {
                MavlinkError::UnsupportedVersion
            }
            mavio::error::SpecError::NotInDialect(id) => MavlinkError::UnknownMessageID(id),
            mavio::error::SpecError::InvalidEnumValue { .. } => MavlinkError::InvalidEnum,
            mavio::error::SpecError::InvalidV1PayloadSize { .. } => MavlinkError::InvalidV1Payload,
            mavio::error::SpecError::V2PayloadIsTooLong { .. } => MavlinkError::V2PayloadIsTooLong,
        }
    }
}

impl From<mavio::error::IoError> for MavlinkError {
    fn from(value: mavio::error::IoError) -> Self {
        match value.kind() {
            mavio::error::IoErrorKind::UnexpectedEof => MavlinkError::UnexpectedEof,
            mavio::error::IoErrorKind::Embedded(kind) => MavlinkError::IoError((*kind).into()),
            mavio::error::IoErrorKind::Generic => MavlinkError::GenericIoError,
        }
    }
}

pub struct Debounce<T> {
    duration: embassy_time::Duration,
    inner: Option<(embassy_time::Instant, T)>,
}

impl<T: PartialEq + Clone> Debounce<T> {
    pub fn new(duration: embassy_time::Duration) -> Self {
        Self {
            duration,
            inner: None,
        }
    }

    pub fn evaluate(&mut self, error: T) -> Option<T> {
        if self
            .inner
            .as_ref()
            .is_none_or(|(t, e)| t.elapsed() > self.duration || e != &error)
        {
            self.inner = Some((embassy_time::Instant::now(), error.clone()));
            Some(error)
        } else {
            None
        }
    }
}
