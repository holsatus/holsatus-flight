#[non_exhaustive]
#[derive(
    serde::Serialize, serde::Deserialize, thiserror::Error, Debug, Clone, Copy, Eq, PartialEq,
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StorageError {
    #[error("The arguments are not properly aligned.")]
    NotAligned,
    #[error("The arguments are out of bounds.")]
    OutOfBounds,
    #[error("Error specific to the implementation.")]
    Other,
}

impl<S: embedded_storage_async::nor_flash::NorFlashError> From<S> for StorageError {
    fn from(value: S) -> Self {
        match value.kind() {
            embedded_storage_async::nor_flash::NorFlashErrorKind::NotAligned => {
                StorageError::NotAligned
            }
            embedded_storage_async::nor_flash::NorFlashErrorKind::OutOfBounds => {
                StorageError::OutOfBounds
            }
            embedded_storage_async::nor_flash::NorFlashErrorKind::Other => StorageError::Other,
            _ => StorageError::Other,
        }
    }
}
