use serde::{Deserialize, Serialize};
use thiserror::Error;

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedI2cError {
    #[error("Bus error occurred. e.g. A START or a STOP condition is detected and is not located after a multiple of 9 SCL clock pulses.")]
    Bus,
    #[error("The arbitration was lost, e.g. electrical problems with the clock signal.")]
    ArbitrationLoss,
    #[error("A bus operation was not acknowledged, e.g. due to the addressed device not being available on the bus or the device not being ready to process requests at the moment.")]
    NoAcknowledge,
    #[error("The peripheral receive buffer was overrun.")]
    Overrun,
    #[error("A different error occurred. ")]
    Other,
}

impl<E: embedded_hal::i2c::Error> From<E> for EmbeddedI2cError {
    fn from(value: E) -> Self {
        use embedded_hal::i2c::ErrorKind as E;
        match value.kind() {
            E::Bus => Self::Bus,
            E::ArbitrationLoss => Self::ArbitrationLoss,
            E::NoAcknowledge(_) => Self::NoAcknowledge,
            E::Overrun => Self::Overrun,
            E::Other => Self::Other,
            _ => Self::Other,
        }
    }
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedSpiError {
    #[error("The peripheral receive buffer was overrun.")]
    Overrun,
    #[error("Multiple devices on the SPI bus are trying to drive the slave select pin, e.g. in a multi-master setup.")]
    ModeFault,
    #[error("Received data does not conform to the peripheral configuration.")]
    FrameFormat,
    #[error("An error occurred while asserting or deasserting the Chip Select pin.")]
    ChipSelectFault,
    #[error("A different error occurred. The original error may contain more information.")]
    Other,
}

impl<E: embedded_hal::spi::Error> From<E> for EmbeddedSpiError {
    fn from(value: E) -> Self {
        use embedded_hal::spi::ErrorKind as E;
        match value.kind() {
            E::Overrun => Self::Overrun,
            E::ModeFault => Self::ModeFault,
            E::FrameFormat => Self::FrameFormat,
            E::ChipSelectFault => Self::ChipSelectFault,
            E::Other => Self::Other,
            _ => Self::Other,
        }
    }
}

#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedDigError {
    #[error("A different error occurred. The original error may contain more information.")]
    Other,
}

impl<E: embedded_hal::digital::Error> From<E> for EmbeddedDigError {
    fn from(value: E) -> Self {
        match value.kind() {
            _ => Self::Other,
        }
    }
}
