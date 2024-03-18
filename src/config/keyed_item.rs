//! This module provides a `KeyedItem` struct that can be used to store data in flash memory
//! when using the `sequential_storage` module. The `KeyedItem` struct is a wrapper around
//! a data item, and a key and handles the serializing and deserializing of the data.
//! The key is used to identify the data item in the flash memory.

use sequential_storage::map::StorageItem;

impl <T, K: Clone> KeyedItem<T, K> {
    pub fn new(data: T, key: K) -> Self {
        Self {
            data,
            key,
        }
    }

    pub fn key(&self) -> K {
        self.key.clone()
    }
}

#[repr(C)]
pub struct KeyedItem<T, K> {
    pub data: T,
    key: K,
}

impl <T, K: Eq + Clone> StorageItem for KeyedItem<T, K> {
    type Key = K;

    type Error = ();

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {

        // The size of self (data + key)
        let self_size = core::mem::size_of::<Self>();

        // Ensure buffer is large enough to contain the data
        if buffer.len() < self_size {
            return Err(());
        }

        // Get self as a byte slice
        let data_bytes =
            unsafe { core::slice::from_raw_parts((self as *const Self) as *const u8, self_size) };

        // Copy the data bytes into the buffer
        buffer[..self_size].copy_from_slice(&data_bytes);

        // Return the size of the data
        Ok(self_size)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {  
        // Ensure buffer is large enough to contain the data
        if buffer.len() < core::mem::size_of::<Self>() {
            return Err(());
        }

        // "Deserialize" the slice by reading from pointer
        let data: Self = unsafe { core::ptr::read_unaligned(buffer.as_ptr() as *const _) };

        // Return the data
        Ok(data)
    }

    fn key(&self) -> Self::Key {
        self.key.clone()
    }

    fn deserialize_key_only(buffer: &[u8]) -> Result<Self::Key, Self::Error>
    where
        Self: Sized,
    {
        // Ensure buffer is large enough to contain the data
        if buffer.len() < core::mem::size_of::<Self>() {
            return Err(());
        }

        // "Deserialize" the slice to only reference the data
        let data_ref = unsafe { &*(buffer.as_ptr() as *const Self) };

        // Return the key
        Ok( data_ref.key.clone() )
    }
}
