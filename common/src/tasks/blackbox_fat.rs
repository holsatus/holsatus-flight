use core::future::Future;

use block_device_adapters::BufStream;
pub use block_device_driver::slice_to_blocks_mut;
pub use block_device_driver::BlockDevice;
use embassy_time::Instant;
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async::Write;
use heapless::{String, Vec};
use postcard::to_slice;

use crate::errors::adapter::postcard::PostcardError;
use crate::errors::BlackboxError;
use crate::errors::HolsatusError;
use crate::signals::BLACKBOX_QUEUE;
use crate::types::blackbox::LoggableType;

pub trait Reset {
    fn reset(&mut self) -> impl Future<Output = bool>;
}

pub async fn main<D, const BUFF: usize>(mut device: D) -> !
where
    D: BlockDevice<512> + Reset,
{
    info!("Blackbox task started");

    let mut error_debounce: Option<(HolsatusError, Instant)> = None;
    loop {
        if let Err(error) = blackbox_run::<D, BUFF>(&mut device).await {
            if error_debounce.is_none_or(|(prev_error, time)| {
                prev_error != error || time.elapsed().as_millis() > 1000
            }) {
                info!("Error occurred, resetting device: {}", error);
                error_debounce = Some((error, Instant::now()));
            }
        }
    }
}

async fn blackbox_run<D, const BUFF: usize>(device: &mut D) -> Result<(), HolsatusError>
where
    D: BlockDevice<512> + Reset,
{
    // Ensure device is in a known-good state
    if !device.reset().await {
        Err(BlackboxError::ResetFault)?
    }

    // TODO - reimplement
    // // Messages likely stale at this point, drop them
    // if BLACKBOX_QUEUE.free_capacity() == 0 {
    //     warn!("Channel was full, dropping all messages");
    //     BLACKBOX_QUEUE.clear();
    // }

    // The BufStream holds a small local cache, and is what the filesystem uses directly
    let inner = BufStream::new(device);

    // Setup the filesystem with default options
    let fs = FileSystem::new(inner, FsOptions::new()).await?;

    // Find log file with largest index
    let mut current_file_idx = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        // The log file must be of format xxxxxx.log
        if entry.is_file() {
            (|| {
                let name = entry.short_file_name_as_bytes();
                let base = name.split(|s| s == &b'.').next()?;
                let base_string = core::str::from_utf8(base).ok()?;
                let parsed_idx = base_string.parse::<u16>().ok()?;
                current_file_idx = current_file_idx.max(parsed_idx);
                Some(())
            })();
        }
    }

    // Construct filename string
    use core::fmt::Write;
    let mut name = String::<12>::new();
    core::write!(&mut name, "{:06}.LOG", current_file_idx.wrapping_add(1)).unwrap();
    info!("Saving to log file {}", name.as_str());

    // Create the file
    let mut file = fs.root_dir().create_file(&name).await?;

    let mut write_buf = Vec::<u8, BUFF>::new();
    let mut serde_buf = [0u8; core::mem::size_of::<LoggableType>()];

    loop {
        let item = BLACKBOX_QUEUE.receive().await;

        let mut serde_slice = to_slice(&item, &mut serde_buf).map_err(|err| {
            Into::<PostcardError>::into(err) // Strange error conversion?
        })?;

        // Ensure we get entirety of packet data
        while serde_slice.len() > 0 {
            // Bytes to copy from vec to buffer
            let to_copy = (write_buf.capacity() - write_buf.len()).min(serde_slice.len());

            // Copy bytes and shorten slices
            let res = write_buf.extend_from_slice(&serde_slice[..to_copy]);
            debug_assert!(res.is_ok(), "Failed to extend buffer");
            serde_slice = &mut serde_slice[to_copy..];

            if write_buf.capacity() == write_buf.len() {
                file.write_all(&write_buf).await?;
            }
        }
    }
}
