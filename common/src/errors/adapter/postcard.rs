use serde::{Deserialize, Serialize};
use thiserror::Error;

/// This is the error type used by Postcard
#[non_exhaustive]
#[derive(Error, Debug, Copy, Clone, Eq, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PostcardError {
    #[error("This is a feature that PostCard will never implement")]
    WontImplement,
    #[error("This is a feature that Postcard intends to support, but does not yet")]
    NotYetImplemented,
    #[error("The serialize buffer is full")]
    SerializeBufferFull,
    #[error("The length of a sequence must be known")]
    SerializeSeqLengthUnknown,
    #[error("Hit the end of buffer, expected more data")]
    DeserializeUnexpectedEnd,
    #[error("Found a varint that didn't terminate. Is the usize too big for this platform?")]
    DeserializeBadVarint,
    #[error("Found a bool that wasn't 0 or 1")]
    DeserializeBadBool,
    #[error("Found an invalid unicode char")]
    DeserializeBadChar,
    #[error("Tried to parse invalid utf-8")]
    DeserializeBadUtf8,
    #[error("Found an Option discriminant that wasn't 0 or 1")]
    DeserializeBadOption,
    #[error("Found an enum discriminant that was > u32::max_value()")]
    DeserializeBadEnum,
    #[error("The original data was not well encoded")]
    DeserializeBadEncoding,
    #[error("Bad CRC while deserializing")]
    DeserializeBadCrc,
    #[error("Serde Serialization Error")]
    SerdeSerCustom,
    #[error("Serde Deserialization Error")]
    SerdeDeCustom,
    #[error("Error while processing `collect_str` during serialization")]
    CollectStrError,
    #[error("Some other serialization error occured")]
    Other,
}

impl From<postcard::Error> for PostcardError {
    fn from(value: postcard::Error) -> Self {
        match value {
            postcard::Error::WontImplement => Self::WontImplement,
            postcard::Error::NotYetImplemented => Self::NotYetImplemented,
            postcard::Error::SerializeBufferFull => Self::SerializeBufferFull,
            postcard::Error::SerializeSeqLengthUnknown => Self::SerializeSeqLengthUnknown,
            postcard::Error::DeserializeUnexpectedEnd => Self::DeserializeUnexpectedEnd,
            postcard::Error::DeserializeBadVarint => Self::DeserializeBadVarint,
            postcard::Error::DeserializeBadBool => Self::DeserializeBadBool,
            postcard::Error::DeserializeBadChar => Self::DeserializeBadChar,
            postcard::Error::DeserializeBadUtf8 => Self::DeserializeBadUtf8,
            postcard::Error::DeserializeBadOption => Self::DeserializeBadOption,
            postcard::Error::DeserializeBadEnum => Self::DeserializeBadEnum,
            postcard::Error::DeserializeBadEncoding => Self::DeserializeBadEncoding,
            postcard::Error::DeserializeBadCrc => Self::DeserializeBadCrc,
            postcard::Error::SerdeSerCustom => Self::SerdeSerCustom,
            postcard::Error::SerdeDeCustom => Self::SerdeDeCustom,
            postcard::Error::CollectStrError => Self::CollectStrError,
            _ => Self::Other,
        }
    }
}
