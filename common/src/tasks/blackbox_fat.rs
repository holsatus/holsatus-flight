use core::future::Future;

use block_device_adapters::BufStream;
pub use block_device_driver::slice_to_blocks_mut;
pub use block_device_driver::BlockDevice;
use embassy_time::Instant;
use embassy_time::Timer;
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
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

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use postcard::{from_bytes_cobs, to_slice_cobs};
    use serde::{Deserialize, Serialize};

    /// Mirror of `imu_data::ImuSample` -- the struct logged to SD card at 1 kHz.
    /// Keeping the definition here (rather than sharing it) avoids pulling
    /// device-specific code into common; the encoding contract is what matters.
    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct ImuSample {
        timestamp_us: u64,
        acc: [i16; 3],
        gyr: [i16; 3],
    }

    #[test]
    fn cobs_round_trip_zero_sample() {
        let sample = ImuSample { timestamp_us: 0, acc: [0, 0, 0], gyr: [0, 0, 0] };
        let mut buf = [0u8; 32];
        let encoded = to_slice_cobs(&sample, &mut buf).unwrap();
        std::assert_eq!(*encoded.last().unwrap(), 0u8, "sentinel missing");
        let decoded: ImuSample = from_bytes_cobs(encoded).unwrap();
        std::assert_eq!(decoded, sample);
    }

    #[test]
    fn cobs_round_trip_typical_sample() {
        let sample = ImuSample {
            timestamp_us: 1_234_567_890,
            acc: [1024, -512, 8192],
            gyr: [-1000, 2000, -32768],
        };
        let mut buf = [0u8; 32];
        let encoded = to_slice_cobs(&sample, &mut buf).unwrap();
        let decoded: ImuSample = from_bytes_cobs(encoded).unwrap();
        std::assert_eq!(decoded, sample);
    }

    #[test]
    fn cobs_encoded_fits_in_30_bytes() {
        // Worst case: all fields at maximum magnitude.
        let sample = ImuSample {
            timestamp_us: u64::MAX,
            acc: [i16::MAX, i16::MIN, i16::MAX],
            gyr: [i16::MIN, i16::MAX, i16::MIN],
        };
        let mut buf = [0u8; 32];
        let encoded = to_slice_cobs(&sample, &mut buf).unwrap();
        std::assert!(encoded.len() <= 30, "encoded {} bytes, expected <= 30", encoded.len());
    }

    #[test]
    fn cobs_no_zero_bytes_inside_frame() {
        let sample = ImuSample {
            timestamp_us: 999_999,
            acc: [100, 200, 300],
            gyr: [-100, -200, -300],
        };
        let mut buf = [0u8; 32];
        let encoded = to_slice_cobs(&sample, &mut buf).unwrap();
        let interior = &encoded[..encoded.len() - 1];
        std::assert!(interior.iter().all(|&b| b != 0), "zero byte inside COBS frame");
    }

    #[test]
    fn cobs_multiple_records_independently_decodable() {
        let samples = [
            ImuSample { timestamp_us: 0, acc: [0, 0, 0], gyr: [0, 0, 0] },
            ImuSample { timestamp_us: 1000, acc: [10, -10, 100], gyr: [5, -5, 50] },
            ImuSample {
                timestamp_us: 2000,
                acc: [i16::MAX, i16::MIN, 0],
                gyr: [0, i16::MAX, i16::MIN],
            },
        ];
        // Build a concatenated stream of COBS records.
        let mut stream: Vec<u8> = Vec::new();
        let mut buf = [0u8; 32];
        for s in &samples {
            let encoded = to_slice_cobs(s, &mut buf).unwrap();
            stream.extend_from_slice(encoded);
        }
        // Decode each record by splitting on zero-byte sentinels.
        let mut offset = 0;
        for expected in &samples {
            let end = stream[offset..].iter().position(|&b| b == 0).unwrap();
            let frame = &mut stream[offset..=offset + end].to_vec();
            let got: ImuSample = from_bytes_cobs(frame).unwrap();
            std::assert_eq!(&got, expected);
            offset += end + 1;
        }
        std::assert_eq!(offset, stream.len());
    }
}

async fn blackbox_run<D, const BUFF: usize>(device: &mut D) -> Result<(), HolsatusError>
where
    D: BlockDevice<512> + Reset,
{
    // Ensure device is in a known-good state
    if !device.reset().await {
        Timer::after_secs(1).await;
        Err(BlackboxError::ResetFault)?
    }

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

        let mut serde_slice =
            to_slice(&item, &mut serde_buf).map_err(Into::<PostcardError>::into)?;

        // Ensure we get entirety of packet data
        while serde_slice.len() > 0 {
            // Bytes to copy from vec to buffer
            let write_buf_rem = write_buf.capacity() - write_buf.len();
            let to_copy = write_buf_rem.min(serde_slice.len());

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
