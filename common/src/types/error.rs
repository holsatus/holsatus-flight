use serde::{Deserialize, Serialize};

use crate::parsers::ParserError;

#[non_exhaustive]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterfaceError {
    Timeout,
    Framing,
    Overrun,
    Unknown,
}


impl From<InterfaceError> for HolsatusError {
    fn from(error: InterfaceError) -> Self {
        HolsatusError::Interface(error)
    }
}

#[non_exhaustive]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SensorError {
    Bus,
    Setup,
    Timeout,
    BadData,
    Unknown,
}

impl From<SensorError> for HolsatusError {
    fn from(error: SensorError) -> Self {
        HolsatusError::Sensor(error)
    }
}


#[non_exhaustive]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    InvalidRange,
    InvalidDeadband,
    InvalidRates,
}

impl From<ConfigError> for HolsatusError {
    fn from(error: ConfigError) -> Self {
        HolsatusError::Config(error)
    }
}

#[non_exhaustive]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BlackboxError {
    QueueFull,
    QueueStaleEmptied,
    UnableToFormMessage,
}

impl From<BlackboxError> for HolsatusError {
    fn from(error: BlackboxError) -> Self {
        HolsatusError::Blackbox(error)
    }
}


#[non_exhaustive]
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HolsatusError {
    Interface(InterfaceError),
    Sensor(SensorError),
    Parser(ParserError),
    Config(ConfigError),
    Blackbox(BlackboxError),
}

macro_rules! impl_from_error {
    (
        $(#[$meta:meta])*
        as $super:ident {
            $($error:ident $(,)? )*
        }
    ) => {
        $(
            impl From<$error> for $super {
                fn from(e: $error) -> Self {
                    $super::$error(e)
                }
            }
        )*

        $(#[$meta])*
        pub enum $super {
            $($error($error),)*
        }
    };
}


impl_from_error!(
    #[non_exhaustive]
    #[derive(Debug, Copy, Clone, Serialize, Deserialize)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    as SuperError {
        InterfaceError,
        SensorError,
        ConfigError,
        ParserError,
    }
);
