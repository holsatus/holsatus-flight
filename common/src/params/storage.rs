use core::ops::Range;

use embedded_storage_async::nor_flash::NorFlash;
use mav_param::{Ident, Value};
use sequential_storage::{
    cache::KeyCacheImpl,
    map::{MapConfig, MapItemIter, MapStorage, PostcardValue},
};

use crate::errors::adapter::sequential_storage::SequentialError;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct WrappedValue(pub Value);

impl PostcardValue<'_> for WrappedValue {}

pub struct ParamStorage<S: NorFlash, C: KeyCacheImpl<[u8; 16]>, const N: usize> {
    storage: MapStorage<[u8; 16], S, C>,
    buffer: [u8; N],
}

impl<S: NorFlash, C: KeyCacheImpl<[u8; 16]>, const N: usize> ParamStorage<S, C, N> {
    pub fn new(storage: S, cache: C, range: Range<u32>) -> Self {
        let config = MapConfig::new(range);
        ParamStorage {
            storage: MapStorage::new(storage, config, cache),
            buffer: [0u8; N],
        }
    }

    // TODO - Will be used for loading individual parameters.
    // However, after we load all into ram on boot is it even needed?
    pub async fn _load(&mut self, key: Ident) -> Result<Option<WrappedValue>, SequentialError> {
        let ret = self
            .storage
            .fetch_item(self.buffer.as_mut(), key.as_raw())
            .await?;
        Ok(ret)
    }

    pub async fn save(&mut self, key: Ident, data: Value) -> Result<(), SequentialError> {
        let data = WrappedValue(data);
        self.storage
            .store_item(self.buffer.as_mut(), key.as_raw(), &data)
            .await?;

        Ok(())
    }

    pub async fn load_iter(&mut self) -> Result<MapItemIter<'_, [u8; 16], S, C>, SequentialError> {
        let ret = self.storage.fetch_all_items(self.buffer.as_mut()).await?;
        Ok(ret)
    }
}
