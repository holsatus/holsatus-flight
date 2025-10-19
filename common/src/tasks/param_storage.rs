use core::{
    ops::Range,
};

use embedded_storage_async::nor_flash::NorFlash;
use maitake_sync::{blocking::Mutex, RwLock, RwLockReadGuard, WaitQueue};
use mav_param::{Ident, Value};
use sequential_storage::{
    cache::{KeyCacheImpl, NoCache},
    map::{MapItemIter, SerializationError},
};

use crate::{errors::adapter::sequential_storage::SequentialError, sync::procedure::Procedure};

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    /// Save all parameters for the `Table` with the given name
    SaveParam(Ident),
    SaveTable(&'static str),
    SaveAllTables,

    LoadTable(&'static str),
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    Success,
    Failure,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    #[error("Parameter error: {0}")]
    Param(#[from] ParamError),
    #[error("Storage error: {0}")]
    Seq(#[from] SequentialError),
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParamError {
    #[error("Invalid utf8 itentifier provided")]
    InvalidIdentifier,
    #[error("No table-level fragment specifier (.) found")]
    NoTableFragment,
    #[error("No table matched the given identifier")]
    NoMachingTable,
    #[error("No parameter matched the given identifier or type")]
    NoMachingParam,
}

static PROCEDURE: Procedure<Request, Response, 1> = Procedure::new();

pub async fn send(req: Request) {
    PROCEDURE.send(req).await;
}

pub async fn request(req: Request) -> Option<Response> {
    PROCEDURE.request(req).await
}

pub struct Table<T: ?Sized> {
    pub name: &'static str,
    pub waiters: WaitQueue,
    pub params: RwLock<T>,
}

impl<T: mav_param::Node + 'static> Table<T> {
    pub const fn new(name: &'static str, data: T) -> Self {
        Table {
            name,
            waiters: WaitQueue::new(),
            params: RwLock::new(data),
        }
    }

    pub const fn default(name: &'static str) -> Self
    where
        T: crate::ConstDefault,
    {
        Self::new(name, T::DEFAULT)
    }

    pub async fn read(&'static self) -> RwLockReadGuard<'static, T> {
        if TABLES.insert(self) {
            request(Request::LoadTable(self.name)).await;
        }

        self.params.read().await
    }
}

impl Table<dyn mav_param::Node> {
    pub async fn num_values(&self) -> usize {
        let lock = self.params.read().await;
        mav_param::param_iter(&*lock).count()
    }
}

pub struct TableSet<const N: usize> {
    pub tables: Mutex<heapless::Vec<&'static Table<dyn mav_param::Node>, N>>,
}

pub static TABLES: TableSet<10> = TableSet::new();

impl<const N: usize> TableSet<N> {
    pub const fn new() -> TableSet<N> {
        TableSet {
            tables: Mutex::new(heapless::Vec::new()),
        }
    }

    pub fn get_table(&self, name: &str) -> Option<&'static Table<dyn mav_param::Node>> {
        self.tables
            .with_lock(|tables| tables.iter().find(|table| table.name == name).cloned())
    }

    /// Insert the given `Table` reference into the `TableSet`.
    ///
    /// Returns `true` if the table is inserted, and it is safe to wait for it to be initialized.
    #[must_use]
    pub fn insert(&self, table: &'static Table<dyn mav_param::Node>) -> bool {
        self.tables.with_lock(|tables| {
            if tables.iter().any(|t| core::ptr::addr_eq(*t, table)) {
                return true;
            }

            if tables.push(table).is_err() {
                error!("[configurator] No more room to add table {}", table.name);
                false
            } else {
                true
            }
        })
    }

    pub async fn get_param(&self, raw_ident: &[u8; 16]) -> Result<Value, ParamError> {
        // Ensure the provided raw_identifier can be converted into a valid identifier
        let ident = Ident::try_from(raw_ident).map_err(|_| ParamError::InvalidIdentifier)?;

        // Split the identifier at the first fragment 'table_ident.param_ident'
        let (table_ident, param_ident) = ident
            .as_str()
            .split_once('.')
            .ok_or(ParamError::NoTableFragment)?;

        // Find the table matching the 'table_ident' specifier
        let table = self.tables.with_lock(|tables| {
            tables
                .iter()
                .find(|table| table.name == table_ident)
                .cloned()
                .ok_or(ParamError::NoMachingTable)
        })?;

        // Find the parameter 'param_ident' in the table
        let reader = table.params.read().await;
        let value =
            mav_param::get_value(&*reader, param_ident).ok_or(ParamError::NoMachingParam)?;

        Ok(value)
    }

    pub async fn set_param(&self, raw_ident: &[u8; 16], value: Value) -> Result<Ident, ParamError> {
        // Ensure the provided raw_identifier can be converted into a valid identifier
        let ident = Ident::try_from(raw_ident).map_err(|_| ParamError::InvalidIdentifier)?;

        // Split the identifier at the first fragment 'table_ident.param_ident'
        let (table_ident, param_ident) = ident
            .as_str()
            .split_once('.')
            .ok_or(ParamError::NoTableFragment)?;

        // Find the table matching the 'table_ident' specifier
        let table = self.tables.with_lock(|tables| {
            tables
                .iter()
                .find(|table| table.name == table_ident)
                .cloned()
                .ok_or(ParamError::NoMachingTable)
        })?;

        // Find the parameter 'param_ident' in the table and set its value
        let mut writer = table.params.write().await;
        mav_param::set_value(&mut *writer, param_ident, value).ok_or(ParamError::NoMachingParam)?;

        Ok(ident)
    }
}

struct ParamStorage<F: NorFlash> {
    storage: InnerStorage<F, NoCache, 32>,
}

impl<F: NorFlash> ParamStorage<F> {
    pub fn new(flash: F, range: Range<u32>) -> Self {
        Self {
            storage: InnerStorage::new(flash, NoCache::new(), range),
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            if let Err(error) = self.run_inner().await {
                error!("[param_storage]: Error: {:?}", error);
            }
        }
    }

    pub async fn run_inner(&mut self) -> Result<(), Error> {
        let (request, handle) = PROCEDURE.receive_request().await;
        let result = match request {
            Request::SaveParam(ident) => self.handle_save_param(ident).await,
            Request::SaveTable(table) => self.handle_save_table(table).await,
            Request::SaveAllTables => self.handle_save_all_tables().await,
            Request::LoadTable(table) => self.handle_load_table(table).await,
        };

        // Ensure the requestee is made aware of the failure
        handle.respond(match result {
            Ok(_) => Response::Success,
            Err(_) => Response::Failure,
        });

        result
    }

    /// Save a single parameter to persistent storage
    pub async fn handle_save_param(&mut self, ident: Ident) -> Result<(), Error> {
        let value = TABLES.get_param(ident.as_raw()).await?;
        self.storage.save(ident, value).await?;
        Ok(())
    }

    /// Save an entire table of parameters to persistent storage.
    pub async fn handle_save_table(&mut self, name: &str) -> Result<(), Error> {
        let table = TABLES.get_table(name).ok_or(ParamError::NoMachingTable)?;
        let params = table.params.read().await;

        for result in mav_param::param_iter_named(&*params, name) {
            let param = result.map_err(|_| ParamError::InvalidIdentifier)?;
            self.storage.save(param.ident.clone(), param.value).await?;
        }

        Ok(())
    }

    /// Save all parameters in all tables to persistent storage
    pub async fn handle_save_all_tables(&mut self) -> Result<(), Error> {
        let mut table_iter = 0;
        while let Some(table) = TABLES.tables.with_lock(|t| t.get(table_iter).cloned()) {
            self.handle_load_table(table.name).await?;
            table_iter += 1;
        }

        Ok(())
    }

    /// Load all parameters from persistent storage for a given table
    pub async fn handle_load_table(&mut self, name: &str) -> Result<(), Error> {
        let table = TABLES.get_table(name).ok_or(ParamError::NoMachingTable)?;
        let mut params = table.params.write().await;

        let mut load_iter = self.storage.load_iter().await?;
        while let Some((key, WrappedValue(value))) = load_iter
            .next(&mut [0u8; 32])
            .await
            .map_err(SequentialError::from)?
        {

            // Note: Even though these should be handled as errors, they 
            // should not prevent us from loading valid parameters for this
            // table. There could just be garbage in the flash, but perhaps
            // that should be more loud than just an error message?

            let Ok(ident) = Ident::try_from(&key) else {
                error!("[param_storage] Invalid identifier, {:?}", key);
                continue;
            };

            let Some((table_ident, param_ident)) = ident.as_str().split_once('.') else {
                error!("[param_storage] No table fragment: {:?}", ident.as_str());
                continue;
            };

            if table_ident != table.name {
                continue;
            }

            mav_param::set_value(&mut *params, param_ident, value);
        }

        Ok(())
    }
}

pub async fn entry<F: NorFlash>(flash: F, range: Range<u32>) -> ! {
    ParamStorage::new(flash, range).run().await
}

#[derive(serde::Serialize, serde::Deserialize)]
struct WrappedValue(Value);

impl sequential_storage::map::Value<'_> for WrappedValue {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        match postcard::to_slice(self, buffer) {
            Ok(bytes) => return Ok(bytes.len()),
            Err(error) => Err(match error {
                postcard::Error::SerializeBufferFull => SerializationError::BufferTooSmall,
                _ => SerializationError::InvalidData,
            }),
        }
    }

    fn deserialize_from(buffer: &'_ [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        match postcard::from_bytes(buffer) {
            Ok(param) => return Ok(param),
            _ => Err(SerializationError::InvalidFormat),
        }
    }
}

pub struct InnerStorage<FLASH, CACHE, const N: usize> {
    flash: FLASH,
    cache: CACHE,
    range: Range<u32>,
    buffer: [u8; N],
}

impl<FLASH: NorFlash, CACHE: KeyCacheImpl<[u8; 16]>, const N: usize> InnerStorage<FLASH, CACHE, N> {
    pub fn new(flash: FLASH, cache: CACHE, range: Range<u32>) -> Self {
        InnerStorage {
            flash,
            cache,
            range,
            buffer: [0u8; N],
        }
    }

    // TODO - Will be used for loading individual parameters.
    // However, after we load all into ram on boot is it even needed?
    async fn _load(&mut self, key: Ident) -> Result<Option<WrappedValue>, SequentialError> {
        use sequential_storage::{cache::NoCache, map::fetch_item};
        let ret = fetch_item::<[u8; 16], WrappedValue, _>(
            &mut self.flash,
            self.range.clone(),
            &mut NoCache::new(),
            self.buffer.as_mut(),
            key.as_raw(),
        )
        .await?;

        Ok(ret)
    }

    async fn save(&mut self, key: Ident, data: Value) -> Result<(), SequentialError> {
        let data = WrappedValue(data);
        use sequential_storage::map::store_item;
        let ret = store_item::<[u8; 16], WrappedValue, _>(
            &mut self.flash,
            self.range.clone(),
            &mut self.cache,
            self.buffer.as_mut(),
            key.as_raw(),
            &data,
        )
        .await?;

        Ok(ret)
    }

    async fn load_iter(
        &mut self,
    ) -> Result<MapItemIter<'_, '_, [u8; 16], FLASH, CACHE>, SequentialError> {
        use sequential_storage::map::fetch_all_items;
        let ret = fetch_all_items::<[u8; 16], _, _>(
            &mut self.flash,
            self.range.clone(),
            &mut self.cache,
            self.buffer.as_mut(),
        )
        .await?;

        Ok(ret)
    }
}
