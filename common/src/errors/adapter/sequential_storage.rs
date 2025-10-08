#[non_exhaustive]
#[derive(
    serde::Serialize, serde::Deserialize, thiserror::Error, Debug, Clone, Copy, Eq, PartialEq,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SequentialError {
    #[error("Error in storage {0}")]
    Storage(#[from] super::embedded_storage::StorageError),
    #[error("The item cannot be stored anymore because the storage is full.")]
    FullStorage,
    #[error("It's been detected that the memory is likely corrupted, you may want to erase it.")]
    Corrupted,
    #[error("A provided buffer was to big to be used")]
    BufferTooBig,
    #[error("A provided buffer was to small to be used, {0} bytes are needed")]
    BufferTooSmall(usize),
    #[error("The serialization could not succeed because the data was not in order.")]
    InvalidData,
    #[error("The deserialization could not succeed because the bytes are in an invalid format.")]
    InvalidFormat,
    #[error("The item is too big to fit in the storage, even if empty.")]
    ItemTooBig,
    #[error("There was a problem during serialization of data")]
    Serialization,
    #[error("Some other sequential storage error occured")]
    Other,
}

impl<S: Into<super::embedded_storage::StorageError>> From<sequential_storage::Error<S>>
    for SequentialError
{
    fn from(value: sequential_storage::Error<S>) -> Self {
        match value {
            sequential_storage::Error::Storage { value } => SequentialError::Storage(value.into()),
            sequential_storage::Error::FullStorage => SequentialError::FullStorage,
            sequential_storage::Error::Corrupted {} => SequentialError::Corrupted,
            sequential_storage::Error::BufferTooBig => SequentialError::BufferTooBig,
            sequential_storage::Error::BufferTooSmall(n) => SequentialError::BufferTooSmall(n),
            sequential_storage::Error::SerializationError(..) => SequentialError::Serialization,
            sequential_storage::Error::ItemTooBig => SequentialError::ItemTooBig,
            _ => SequentialError::Other,
        }
    }
}
