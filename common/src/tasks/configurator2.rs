use core::ops::{DerefMut, Range};

use crate::sync::channel::Channel;
use embedded_storage_async::nor_flash::NorFlash;
use maitake_sync::{Mutex, MutexGuard, WaitMap};
use mutex::raw_impls::cs::CriticalSectionRawMutex as M;
use postcard_schema::{key::Key, Schema};
use sequential_storage::map::{SerializationError, Value};
use serde::{de::DeserializeOwned, Serialize};

enum Message {
    Load {
        key: [u8; 8],
        buf: MutexGuard<'static, MiniBuf>,
    },
    Save {
        key: [u8; 8],
        buf: MutexGuard<'static, MiniBuf>,
    },
}

#[derive(PartialEq)]
enum Op {
    Load,
    Save,
}

pub struct Configurator {
    name: &'static str,
    channel: Channel<Message, M, 1>,
    response: WaitMap<([u8; 8], Op), Option<MutexGuard<'static, MiniBuf>>>,
    byte_buf: Mutex<MiniBuf>,
}

impl Default for Configurator {
    fn default() -> Self {
        Configurator::new()
    }
}

impl Configurator {
    pub const fn new() -> Self {
        Self {
            name: "configurator",
            channel: Channel::new(),
            response: WaitMap::new(),
            byte_buf: Mutex::new(MiniBuf::new()),
        }
    }

    /// Load a piece of data from the configurator.
    async fn load_inner(&'static self, uuid: &str) -> Option<MutexGuard<'static, MiniBuf>> {
        trace!("[{}] Trying to load data for key: {}", self.name, uuid);

        // Convert the identifier to a key (using postcard schema)
        let key = Key::for_path::<MiniBuf>(uuid).to_bytes();

        // Lock the global byte buffer mutex (avoids allocation)
        let buf = self.byte_buf.lock().await;

        // Send a load message to the configurator task
        self.channel.send(Message::Load { key, buf }).await;

        // Wait for the configurator to respond
        self.response.wait((key, Op::Load)).await.ok()?
    }

    /// Load a piece of data from the configurator.
    pub async fn load<T: DeserializeOwned>(&'static self, uuid: &str) -> Option<T> {
        // Call the inner function (to reduce monomorphization)
        let buffer = self.load_inner(uuid).await?;

        // Get a slice of the relevant data in the buffer
        let sub_buf = &buffer.data[..buffer.len as usize];

        // Deserialize the buffer into the target type
        postcard::from_bytes::<T>(sub_buf).ok()
    }

    /// Load a piece of data from the configurator, or return the default value if it doesn't exist.
    ///
    /// This is similar to calling `load` and then using `unwrap_or_default`, but this function
    /// also logs the result of the operation using the `trace` macro.
    pub async fn load_or_default<T: DeserializeOwned + Default>(&'static self, uuid: &str) -> T {
        match self.load(uuid).await {
            Some(data) => {
                trace!(
                    "[{}] Found and deserialized data for key: {}",
                    self.name,
                    uuid
                );
                data
            }
            None => {
                trace!(
                    "[{}] No data found for key: {}, using default",
                    self.name,
                    uuid
                );
                Default::default()
            }
        }
    }

    /// Save a piece of data to the configurators storage.
    pub async fn save<T: Serialize>(&'static self, uuid: &str, data: T) -> Result<(), ()> {
        trace!("[{}] Trying to savie data for key: {}", self.name, uuid);

        // Convert the identifier to a key (using postcard schema)
        let key = Key::for_path::<MiniBuf>(uuid).to_bytes();

        // Lock the global byte buffer mutex (avoids allocation)
        let mut buf = self.byte_buf.lock().await;

        // Serialize the data into the buffer
        match postcard::to_slice(&data, &mut buf.data) {
            Ok(serde) => buf.len = serde.len() as u8,
            Err(_) => {
                error!("[{}] Failed to serialize data for key: {}", self.name, uuid);
                return Err(());
            }
        }

        // Send a save message to the configurator
        self.channel.send(Message::Save { key, buf }).await;

        // Wait for the configurator to respond
        match self.response.wait((key, Op::Save)).await {
            Ok(_) => Ok(()),
            _ => Err(()),
        }
    }
}

pub async fn load_or_default<T: Default + DeserializeOwned>(uuid: &str) -> T {
    CONFIGURATOR.load_or_default(uuid).await
}

pub static CONFIGURATOR: Configurator = Configurator::new();

pub async fn configurator_entry(
    flash: impl NorFlash,
    range: Range<u32>,
) -> ! {
    // Create the storage instance with a buffer slightly
    // larger than the maximum size of the MiniBuf struct.
    let mut storage = Storage::<_, { 256 + 8 }>::new(flash, range);

    loop {
        match CONFIGURATOR.channel.receive().await {
            Message::Load { key, mut buf } => match storage.load(&key).await {
                Ok(Some(data)) => {
                    *buf.deref_mut() = data;
                    debug!("[{}] Loaded data for key: {:?}", CONFIGURATOR.name, key);
                    CONFIGURATOR.response.wake(&(key, Op::Load), Some(buf));
                }
                _ => {
                    error!("[{}] Failed to load data for key: {:?}", CONFIGURATOR.name, key);
                    CONFIGURATOR.response.wake(&(key, Op::Load), None);
                }
            },
            Message::Save { key, mut buf } => {
                let deref = buf.deref_mut();
                match storage.save(&key, deref).await {
                    Ok(()) => {
                        CONFIGURATOR.response.wake(&(key, Op::Save), None);
                        debug!("[{}] Saved data for key: {:?}", CONFIGURATOR.name, key);
                    }
                    _ => {
                        CONFIGURATOR.response.wake(&(key, Op::Save), None);
                        error!("[{}] Failed to save data for key: {:?}", CONFIGURATOR.name, key);
                    }
                }
            }
        }
    }
}

#[derive(Schema)]
struct MiniBuf {
    data: [u8; 255],
    len: u8,
}

impl MiniBuf {
    pub const fn new() -> Self {
        Self {
            data: [0u8; 0xFF],
            len: 0,
        }
    }
}

impl Value<'_> for MiniBuf {
    fn serialize_into(&self, mut buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // Ensure the buffer is large enough
        let len = self.len as usize;
        if buffer.len() < 1 + len {
            return Err(SerializationError::BufferTooSmall);
        }

        // Get first byte, reduce buffer size
        buffer[0] = self.len;
        buffer = &mut buffer[1..];

        // Copy data into buffer
        buffer[..len].copy_from_slice(&self.data[..len]);

        // Return the number of bytes serialized
        Ok(1 + len)
    }

    fn deserialize_from(mut buffer: &'_ [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 1 {
            return Err(SerializationError::BufferTooSmall);
        }

        let len = buffer[0];
        buffer = &buffer[1..];

        if buffer.len() < len as usize {
            return Err(SerializationError::InvalidFormat);
        }

        let mut data = [0u8; 255];
        data[..len as usize].copy_from_slice(&buffer[..len as usize]);

        Ok(Self { data, len })
    }
}

pub struct Storage<FLASH, const N: usize> {
    flash: FLASH,
    range: Range<u32>,
    buffer: [u8; N],
}

impl<FLASH: NorFlash, const N: usize> Storage<FLASH, N> {
    pub fn new(flash: FLASH, range: Range<u32>) -> Self {
        Storage {
            flash,
            range,
            buffer: [0u8; N],
        }
    }

    async fn load(
        &mut self,
        key: &[u8; 8],
    ) -> Result<Option<MiniBuf>, crate::errors::adapter::sequential_storage::SequentialError> {
        use sequential_storage::{cache::NoCache, map::fetch_item};
        fetch_item::<[u8; 8], MiniBuf, _>(
            &mut self.flash,
            self.range.clone(),
            &mut NoCache::new(),
            self.buffer.as_mut(),
            key,
        )
        .await
        .map_err(|_| crate::errors::adapter::sequential_storage::SequentialError::Other)
    }

    async fn save(
        &mut self,
        key: &[u8; 8],
        data: &MiniBuf,
    ) -> Result<(), crate::errors::adapter::sequential_storage::SequentialError> {
        use sequential_storage::{cache::NoCache, map::store_item};
        store_item::<[u8; 8], MiniBuf, _>(
            &mut self.flash,
            self.range.clone(),
            &mut NoCache::new(),
            self.buffer.as_mut(),
            key,
            data,
        )
        .await
        .map_err(|_| crate::errors::adapter::sequential_storage::SequentialError::Other)
    }
}
