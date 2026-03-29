//! MicoAir H743v2 -- IMU data logger to SD card.
//!
//! Reads BMI088/BMI088_MM at 1 kHz over SPI2 and writes COBS+postcard records
//! to a sequentially-named `.IMU` file on SDMMC1.  UART1 prints a live summary
//! at 20 Hz so you can confirm the sensor is working without pulling the SD card.
//!
//! # Hardware
//!   BMI088 accel SPI2: SCLK=PD3  MOSI=PC3  MISO=PC2  CS=PD4
//!   BMI088 gyro  SPI2: SCLK=PD3  MOSI=PC3  MISO=PC2  CS=PD5
//!   SDMMC1:            CLK=PC12  CMD=PD2   D0-D3=PC8-PC11
//!   UART1:             TX=PA9  (115 200 baud)
//!
//! # Log format
//!   Directory  : D000001/, D000002/, ... (one per boot, 8.3-compatible name)
//!   File name  : D000001/000001.IMU, 000002.IMU, ... (rotates every 30 s)
//!   Each record: COBS-encoded postcard blob, zero-byte terminated.
//!   Struct     : { timestamp_us: u64, acc: [i16; 3], gyr: [i16; 3] }
//!   Accel scale: +/-6 g range  -> 1 g = 5460.8 LSB
//!   Gyro  scale: +/-2000 dps   -> 1 dps = 16.384 LSB
//!
//! # Copying the log to macOS
//!   Pop the SD card and insert it.  It mounts as a FAT volume.
//!   Copy the entire session directory with Finder or:
//!     cp -r /Volumes/<card>/data_000001 ~/Desktop/
//!
//! # Decoding on macOS / Linux
//!   pip install cobs postcard-python  (or just use the struct layout)
//!
//!   Concatenate all rotations in one session then decode:
//!     cat data_000001/*.IMU > session.bin
//!
//!   Python decoder:
//!     import cobs.cobs as cobs, struct, pathlib
//!     raw = pathlib.Path("session.bin").read_bytes()
//!     packets = [p for p in raw.split(b'\x00') if p]
//!     records = []
//!     for pkt in packets:
//!         dec = cobs.decode(pkt)
//!         # postcard varint u64 + 6x zigzag i16
//!         # quickest approach: use the `postcard` Python crate or decode manually.
//!         # Manual layout (typical small values):
//!         #   byte 0     : timestamp low 7 bits | 0x80 if more follows (varint)
//!         #   bytes 1..N : more varint bytes until MSB=0
//!         #   then 6x i16 as zigzag varints (LSB=sign, rest=magnitude>>1)
//!         records.append(dec)
//!
//!   Alternatively install the `postcard` Rust tool to pretty-print records.

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::spi::{self, mode::Master, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::{String, Vec};
use postcard::to_slice_cobs;
use serde::Serialize;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use common::tasks::blackbox_fat::Reset;
use micoairh743v2::bmi088::Bmi088;
use micoairh743v2::resources::Spi2Irqs;
use micoairh743v2::sdlog::SdmmcResources;

// ── Log record ───────────────────────────────────────────────────────────────

/// One IMU sample as logged to SD card.
///
/// Postcard encodes integers as varints; at full-scale the struct serialises
/// to at most 28 bytes.  With COBS overhead and sentinel the worst case is
/// 30 bytes per record, giving ~30 kB/s at 1 kHz.
#[derive(Serialize)]
struct ImuSample {
    timestamp_us: u64,
    acc: [i16; 3],
    gyr: [i16; 3],
}

// ── Interrupt bindings ───────────────────────────────────────────────────────

// DMA1_STREAM6 and DMA1_STREAM7 are bound at lib level (resources::Spi2Irqs)
// to avoid duplicate #[no_mangle] symbols when the lib is linked in.
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;   // UART1 TX DMA
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();

// ── Entry point ──────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    // Three green blinks: board alive.
    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    // ── UART ────────────────────────────────────────────────────────────────
    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default()).unwrap();
    uart.write(b"imu_log: UART ok\r\n").await.ok();

    // ── SPI2 / BMI088 ───────────────────────────────────────────────────────
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(8_000_000);
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;

    let spi = Spi::new(
        p.SPI2, p.PD3, p.PC3, p.PC2,
        p.DMA1_CH6, p.DMA1_CH7,
        Spi2Irqs, spi_cfg,
    );
    let bus = SPI2_BUS.init(Mutex::new(spi));
    let accel_dev = SpiDeviceWithConfig::new(bus, Output::new(p.PD4, Level::High, Speed::High), spi_cfg);
    let gyro_dev  = SpiDeviceWithConfig::new(bus, Output::new(p.PD5, Level::High, Speed::High), spi_cfg);
    let mut imu = Bmi088::new(accel_dev, gyro_dev);

    led_blue.set_high();
    match imu.init().await {
        Ok(()) => {
            led_blue.set_low();
            uart.write(b"imu_log: BMI088 ok\r\n").await.ok();
        }
        Err(e) => {
            led_blue.set_low();
            let mut s: String<48> = String::new();
            match e {
                micoairh743v2::bmi088::Error::WrongAccelId(id) =>
                    write!(s, "imu_log: BMI088 accel id=0x{:02X}\r\n", id).ok(),
                micoairh743v2::bmi088::Error::WrongGyroId(id) =>
                    write!(s, "imu_log: BMI088 gyro  id=0x{:02X}\r\n", id).ok(),
                micoairh743v2::bmi088::Error::Spi =>
                    write!(s, "imu_log: BMI088 SPI err\r\n").ok(),
            };
            uart.write(s.as_bytes()).await.ok();
            loop {
                led_red.set_high();
                Timer::after_millis(200).await;
                led_red.set_low();
                Timer::after_millis(200).await;
            }
        }
    }

    // ── SDMMC1 ──────────────────────────────────────────────────────────────
    let mut device = SdmmcResources {
        periph: p.SDMMC1,
        clk: p.PC12,
        cmd: p.PD2,
        d0: p.PC8,
        d1: p.PC9,
        d2: p.PC10,
        d3: p.PC11,
    }
    .setup();

    if !device.reset().await {
        uart.write(b"imu_log: SD card init FAIL\r\n").await.ok();
        loop {
            for _ in 0..4u8 {
                led_red.set_high();
                Timer::after_millis(150).await;
                led_red.set_low();
                Timer::after_millis(150).await;
            }
            Timer::after_millis(800).await;
        }
    }
    uart.write(b"imu_log: SD ok\r\n").await.ok();

    // ── FAT filesystem ───────────────────────────────────────────────────────
    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"imu_log: FAT mount FAIL\r\n").await.ok();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    uart.write(b"imu_log: FAT ok\r\n").await.ok();

    // Write an initial LOG.TXT so that mounting the card immediately after
    // boot shows at least "sd:ok fat:ok" even before any .IMU file is created.
    write_status_log(&fs, "sd:ok fat:ok", "", 0, 0).await;

    // Scan root for D%06u directories to find the next session index.
    // Using 8.3-compatible names (D000001 = 7 chars) avoids LFN and the
    // `alloc` feature requirement in embedded-fatfs.
    let mut session_idx: u32 = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        if entry.is_dir() {
            if let Some(idx) = parse_session_dir_idx(entry.short_file_name_as_bytes()) {
                session_idx = session_idx.max(idx);
            }
        }
    }
    session_idx += 1;

    let mut dir_name: String<8> = String::new();
    write!(dir_name, "D{:06}", session_idx).ok();

    let mut msg: String<32> = String::new();
    write!(msg, "imu_log: session {}\r\n", dir_name.as_str()).ok();
    uart.write(msg.as_bytes()).await.ok();

    let session_dir = match fs.root_dir().create_dir(dir_name.as_str()).await {
        Ok(d) => d,
        Err(_) => {
            uart.write(b"imu_log: session dir FAIL\r\n").await.ok();
            write_status_log(&fs, "sd:ok fat:ok dir:FAIL", dir_name.as_str(), 0, 0).await;
            loop { Timer::after_secs(1).await; }
        }
    };

    let mut file_idx: u16 = 1;
    let mut fname: String<12> = String::new();
    write!(fname, "{:06}.IMU", file_idx).ok();

    let mut file = match session_dir.create_file(fname.as_str()).await {
        Ok(f) => {
            let mut m: String<48> = String::new();
            write!(m, "imu_log: {}/{} open ok\r\n", dir_name.as_str(), fname.as_str()).ok();
            uart.write(m.as_bytes()).await.ok();
            f
        }
        Err(_) => {
            uart.write(b"imu_log: file create FAIL\r\n").await.ok();
            write_status_log(&fs, "sd:ok fat:ok file:FAIL", fname.as_str(), 0, 0).await;
            loop { Timer::after_secs(1).await; }
        }
    };

    // Update LOG.TXT now that the first .IMU file is open.
    write_status_log(&fs, "sd:ok fat:ok file:open", fname.as_str(), 0, 0).await;
    uart.write(b"imu_log: logging started\r\n").await.ok();

    // ── Logging loop ─────────────────────────────────────────────────────────
    //
    // serde_buf: scratch for one COBS-framed record.
    //   Worst-case postcard size for ImuSample: 10 (u64 varint) + 9 + 9 = 28 B.
    //   COBS overhead: ceil(28/254) = 1 B.  Sentinel: 1 B.  Total <= 30 B.
    //   Use 32 bytes.
    //
    // write_buf: accumulates records between FAT writes.
    //   At 1 kHz with ~30 B/record the 4 KB buffer fills in ~136 ms.  An
    //   additional timer-driven flush fires at 500 ms to drain any partial
    //   buffer, ensuring data is safe on the card even after a crash.
    //
    // File rotation: FAT only updates a file's size field in the directory
    //   entry when the file is closed.  Power-cut without close => the file
    //   appears as 0 bytes on the host even though the cluster data is on
    //   disk.  Rotating every ROTATE_SECS closes and finalises the previous
    //   file, so at worst only the current (open) rotation is empty.
    let mut serde_buf = [0u8; 32];
    let mut write_buf: Vec<u8, 4096> = Vec::new();
    let mut last_flush = Instant::now();
    let mut last_rotate = Instant::now();
    let mut ticker = Ticker::every(Duration::from_hz(1000));
    let mut uart_divider: u32 = 0;
    let mut imu_bytes_total: u32 = 0;
    let mut rotation_count: u16 = 0;

    const ROTATE_SECS: u64 = 30;

    led_green.set_high();

    loop {
        ticker.next().await;

        let Ok(d) = imu.read().await else { continue };

        let sample = ImuSample {
            timestamp_us: Instant::now().as_micros(),
            acc: [d.accel.x, d.accel.y, d.accel.z],
            gyr: [d.gyro.x, d.gyro.y, d.gyro.z],
        };

        if let Ok(encoded) = to_slice_cobs(&sample, &mut serde_buf) {
            if write_buf.extend_from_slice(encoded).is_err() {
                // Buffer full -- flush synchronously then append.
                led_blue.set_high();
                if file.write_all(&write_buf).await.is_ok() {
                    imu_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
                led_blue.set_low();
                write_buf.extend_from_slice(encoded).ok();
            }
        }

        // Timer-driven flush: guarantees data on card within 500 ms.
        // flush() also updates the FAT directory entry (file size) so the
        // file is never 0 bytes on the host even after an ungraceful power-off.
        if last_flush.elapsed() > Duration::from_millis(500) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    imu_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            file.flush().await.ok();
            led_blue.set_low();
            last_flush = Instant::now();
        }

        // File rotation: close the current file (finalises the FAT directory
        // entry) and open the next one.  After this the completed file is
        // fully readable even if power is cut immediately afterwards.
        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    imu_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            drop(file);
            rotation_count += 1;

            let mut m: String<48> = String::new();
            write!(m, "imu_log: closed {} ({} B)\r\n", fname.as_str(), imu_bytes_total).ok();
            uart.write(m.as_bytes()).await.ok();

            file_idx = file_idx.wrapping_add(1);
            fname.clear();
            write!(fname, "{:06}.IMU", file_idx).ok();

            file = match session_dir.create_file(fname.as_str()).await {
                Ok(f) => f,
                Err(_) => {
                    uart.write(b"imu_log: rotate FAIL\r\n").await.ok();
                    write_status_log(&fs, "sd:ok fat:ok file:rotate-FAIL", fname.as_str(), imu_bytes_total, rotation_count).await;
                    loop { Timer::after_secs(1).await; }
                }
            };
            led_blue.set_low();

            // Update LOG.TXT: the just-closed file is now permanently readable.
            write_status_log(&fs, "sd:ok fat:ok file:open", fname.as_str(), imu_bytes_total, rotation_count).await;

            let mut msg2: String<48> = String::new();
            write!(msg2, "imu_log: -> {}\r\n", fname.as_str()).ok();
            uart.write(msg2.as_bytes()).await.ok();

            last_rotate = Instant::now();
            last_flush = Instant::now();
        }

        // UART summary at 20 Hz (every 50 ticks).
        uart_divider += 1;
        if uart_divider >= 50 {
            uart_divider = 0;
            let mut s: String<96> = String::new();
            write!(
                s,
                "ax={} ay={} az={} gx={} gy={} gz={}\r\n",
                sample.acc[0], sample.acc[1], sample.acc[2],
                sample.gyr[0], sample.gyr[1], sample.gyr[2],
            )
            .ok();
            uart.write(s.as_bytes()).await.ok();
        }
    }
}

/// Write (overwrite) LOG.TXT with current status.
///
/// The file is created and immediately dropped (closed), so the FAT directory
/// entry is always finalised.  Safe to call even if the current .IMU file is
/// open -- FAT supports multiple file handles.
async fn write_status_log<IO, TP, OCC>(
    fs: &FileSystem<IO, TP, OCC>,
    status: &str,
    current_file: &str,
    bytes_total: u32,
    rotations: u16,
) where
    IO: embedded_io_async_061::Read + embedded_io_async_061::Write + embedded_io_async_061::Seek,
    TP: embedded_fatfs::TimeProvider,
    OCC: embedded_fatfs::OemCpConverter,
{
    let mut content: String<160> = String::new();
    write!(content, "status: {}\r\n", status).ok();
    if !current_file.is_empty() {
        write!(content, "file: {}\r\n", current_file).ok();
    }
    write!(content, "bytes: {}\r\n", bytes_total).ok();
    write!(content, "rotations: {}\r\n", rotations).ok();

    if let Ok(mut lf) = fs.root_dir().create_file("LOG.TXT").await {
        lf.write_all(content.as_bytes()).await.ok();
        // drop closes the file and finalises the directory entry
    }
}

/// Parse the session index from the short (8.3) name of a `D%06u` directory.
///
/// `name` is from `short_file_name_as_bytes()`. Accepts both `D` and `d`.
/// Returns `None` if the name does not match `D\d{6}` exactly.
fn parse_session_dir_idx(name: &[u8]) -> Option<u32> {
    if name.len() != 7 {
        return None;
    }
    if name[0] != b'D' && name[0] != b'd' {
        return None;
    }
    let mut idx: u32 = 0;
    for &b in &name[1..] {
        if b < b'0' || b > b'9' {
            return None;
        }
        idx = idx * 10 + (b - b'0') as u32;
    }
    Some(idx)
}
