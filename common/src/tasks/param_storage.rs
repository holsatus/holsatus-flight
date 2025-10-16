use core::{
    ops::Range,
    sync::atomic::{AtomicUsize, Ordering},
};

use embassy_time::Timer;
use embedded_storage_async::nor_flash::NorFlash;
use maitake_sync::{blocking::Mutex, RwLock, RwLockReadGuard, WaitQueue};
use mav_param::Value;
use sequential_storage::{
    cache::{KeyCacheImpl, NoCache},
    map::{MapItemIter, SerializationError},
};

use crate::{errors::adapter::sequential_storage::SequentialError, sync::procedure::Procedure};

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Request {
    /// Save all parameters for the `Table` with the given name
    SaveTable(&'static str),
    SaveAllTables,
    SaveParam(mav_param::Ident),
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    Success,
    Failure,
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
    pub generation: AtomicUsize,
    pub waiters: WaitQueue,
    pub params: RwLock<T>,
}

impl<T: mav_param::Node + 'static> Table<T> {
    pub const fn new(name: &'static str, data: T) -> Self {
        Table {
            name,
            generation: AtomicUsize::new(0),
            waiters: WaitQueue::new(),
            params: RwLock::new(data),
        }
    }

    pub async fn read_initialized(&'static self) -> RwLockReadGuard<'static, T> {
        if TABLES.insert(self) {
            self.wait_updated(&mut 0).await;
        }

        self.params.read().await
    }

    pub async fn wait_updated(&'static self, generation: &mut usize) {
        let res = self
            .waiters
            .wait_for(|| {
                let loaded = self.generation.load(Ordering::Acquire);
                if loaded > *generation {
                    *generation = loaded;
                    true
                } else {
                    false
                }
            })
            .await;

        debug_assert!(res.is_ok());
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

            let result = tables.push(table);

            if result.is_ok() {
                debug!("Table '{}' has been inserted in global list", table.name);
            } else {
                error!(
                    "[configurator] No room to add table {}, supports maximum of {}",
                    table.name, N
                )
            }

            result.is_ok()
        })
    }

    pub async fn get_param(&self, ident: [u8; 16]) -> Option<Value> {
        let Ok(ident) = mav_param::Ident::try_from(&ident) else {
            error!("[configurator] Key not valid identifier: {:?}", ident);
            return None;
        };

        let Some((first, last)) = ident.as_str().split_once('.') else {
            error!("[configurator] Expected '.' in ident: {}", ident.as_str());
            return None;
        };

        let table = self.tables.with_lock(|tables| {
            let maybe_table = tables.iter().find(|table| table.name == first);
            if maybe_table.is_none() {
                error!("[configurator] No table with the name: {}", first);
            }
            maybe_table.cloned()
        })?;

        let reader = table.params.read().await;
        let Some(value) = mav_param::get_value(&*reader, last) else {
            error!("[configurator] No such parameter: {}.{}", first, last);
            return None;
        };

        Some(value)
    }

    pub async fn set_param(&self, ident: [u8; 16], value: Value) -> Option<mav_param::Ident> {
        let Ok(ident) = mav_param::Ident::try_from(&ident) else {
            error!("[configurator] Key not valid identifier: {:?}", ident);
            return None;
        };

        let Some((first, last)) = ident.as_str().split_once('.') else {
            error!("[configurator] Expected '.' in ident: {}", ident.as_str());
            return None;
        };

        let table = self.tables.with_lock(|tables| {
            let maybe_table = tables.iter().find(|table| table.name == first);
            if maybe_table.is_none() {
                error!("[configurator] No table with the name: {}", first);
            }
            maybe_table.cloned()
        })?;

        let mut writer = table.params.write().await;
        let Some(()) = mav_param::set_value(&mut *writer, last, value) else {
            error!("[configurator] No such parameter: {}.{}", first, last);
            return None;
        };

        Some(ident)
    }
}

pub async fn param_storage(flash: impl NorFlash, range: Range<u32>) -> ! {
    let mut storage = ParamStorage::<_, _, 32>::new(flash, NoCache::new(), range);

    // Allow all tasks to register themselves in list
    Timer::after_millis(100).await;

    match storage.load_iter().await {
        Ok(mut iter) => loop {
            match iter.next::<WrappedValue>(&mut [0u8; 32]).await {
                Ok(Some((key, WrappedValue(value)))) => {
                    debug!("[configurator] Loaded parameter: {:?}: {:?}", key, value);
                    TABLES.set_param(key, value).await;
                }
                Ok(None) => break,
                Err(_) => {
                    error!("[configurator] Error loading parameter, maybe storage is tainted? (2)")
                }
            }
        },
        Err(_) => error!("[configurator] Error loading parameter, maybe storage is tainted? (1)"),
    }

    // Wake anyone waiting for tables to be loaded
    debug!("[configurator] Waking anyone waiting for tables");
    TABLES.tables.with_lock(|tables| {
        for table in tables {
            table.generation.fetch_add(1, Ordering::AcqRel);
            table.waiters.wake_all();
        }
    });

    loop {
        let (request, handle) = PROCEDURE.receive_request().await;
        match request {
            Request::SaveTable(name) => {
                let Some(table) = TABLES.get_table(name) else {
                    error!(
                        "[configurator] No table named '{}' exists, discarding SaveTable command",
                        name
                    );
                    handle.respond(Response::Failure);
                    continue;
                };

                let params = table.params.read().await;

                let mut error_occured = false;
                for result in mav_param::param_iter_named(&*params, name) {
                    match result {
                        Ok(param) => {
                            if storage
                                .save(param.ident.clone(), param.value)
                                .await
                                .is_err()
                            {
                                error!(
                                    "[configurator] Could not save parameter: {}",
                                    param.ident.as_str()
                                );
                                error_occured = true;
                            }
                        }
                        Err(_) => {
                            error!(
                                "[configurator] Could not construct a parameter in table '{}'",
                                name
                            );
                            error_occured = true;
                        }
                    }
                }

                handle.respond(match error_occured {
                    true => Response::Failure,
                    false => Response::Success,
                })
            }
            Request::SaveParam(ident) => {
                let Some((module, param)) = ident.as_str().split_once('.') else {
                    error!(
                        "[configurator] The identifier {:?} does not specify a root module",
                        ident
                    );
                    handle.respond(Response::Failure);
                    continue;
                };

                let Some(table) = TABLES.get_table(module) else {
                    error!("[configurator] No table named '{}' exists", module);
                    handle.respond(Response::Failure);
                    continue;
                };

                let params = table.params.read().await;

                if let Some(value) = mav_param::get_value(&*params, param) {
                    match storage.save(ident.clone(), value).await {
                        Ok(_) => {
                            handle.respond(Response::Success);
                            info!(
                                "[configurator] Saved parameter {:?} to persistent storage",
                                ident
                            )
                        }
                        Err(_) => {
                            handle.respond(Response::Failure);
                            error!("[configurator] Error while saving to persistent storage")
                        }
                    }
                } else {
                    handle.respond(Response::Failure);
                    error!("[configurator] No parameter for {:?}", ident)
                }
            }
            Request::SaveAllTables => {
                let tables = TABLES.tables.with_lock(|t| t.clone());

                let mut error_occured = false;
                for table in tables {
                    let params = table.params.read().await;

                    for result in mav_param::param_iter_named(&*params, table.name) {
                        match result {
                            Ok(param) => {
                                if storage
                                    .save(param.ident.clone(), param.value)
                                    .await
                                    .is_err()
                                {
                                    error!(
                                        "[configurator] Could not save parameter: {}",
                                        param.ident.as_str()
                                    );
                                    error_occured = true;
                                }
                            }
                            Err(_) => {
                                error!(
                                    "[configurator] Could not construct a parameter in table '{}'",
                                    table.name
                                );
                                error_occured = true;
                            }
                        }
                    }
                }

                handle.respond(match error_occured {
                    true => Response::Failure,
                    false => Response::Success,
                })
            }
        }
    }
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

pub struct ParamStorage<FLASH, CACHE, const N: usize> {
    flash: FLASH,
    cache: CACHE,
    range: Range<u32>,
    buffer: [u8; N],
}

impl<FLASH: NorFlash, CACHE: KeyCacheImpl<[u8; 16]>, const N: usize> ParamStorage<FLASH, CACHE, N> {
    pub fn new(flash: FLASH, cache: CACHE, range: Range<u32>) -> Self {
        ParamStorage {
            flash,
            cache,
            range,
            buffer: [0u8; N],
        }
    }

    // TODO - Will be used for loading individual parameters.
    // However, after we load all into ram on boot is it even needed?
    async fn _load(
        &mut self,
        key: mav_param::Ident,
    ) -> Result<Option<WrappedValue>, SequentialError> {
        use sequential_storage::{cache::NoCache, map::fetch_item};
        fetch_item::<[u8; 16], WrappedValue, _>(
            &mut self.flash,
            self.range.clone(),
            &mut NoCache::new(),
            self.buffer.as_mut(),
            key.as_raw(),
        )
        .await
        .map_err(|_| SequentialError::Corrupted)
    }

    async fn save(&mut self, key: mav_param::Ident, data: Value) -> Result<(), SequentialError> {
        let data = WrappedValue(data);

        use sequential_storage::map::store_item;
        store_item::<[u8; 16], WrappedValue, _>(
            &mut self.flash,
            self.range.clone(),
            &mut self.cache,
            self.buffer.as_mut(),
            key.as_raw(),
            &data,
        )
        .await
        .map_err(|_| SequentialError::Corrupted)
    }

    async fn load_iter(
        &mut self,
    ) -> Result<MapItemIter<'_, '_, [u8; 16], FLASH, CACHE>, SequentialError> {
        use sequential_storage::map::fetch_all_items;
        fetch_all_items::<[u8; 16], _, _>(
            &mut self.flash,
            self.range.clone(),
            &mut self.cache,
            self.buffer.as_mut(),
        )
        .await
        .map_err(|_| SequentialError::Corrupted)
    }
}
