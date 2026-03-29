//! MicoAir H743v2 -- IST8310 compass data logger to SD card.
//!
//! Samples IST8310 at 10 Hz over I2C2 and writes COBS+postcard records to a
//! sequentially-named `.MAG` file inside the current session directory on
//! SDMMC1. UART1 prints a live reading once per sample.
//!
//! # Hardware
//!   IST8310 I2C2:  SCL=PB10  SDA=PB11  addr=0x0E (fixed)
//!   SDMMC1:        CLK=PC12  CMD=PD2   D0-D3=PC8-PC11
//!   UART1:         TX=PA9  (115 200 baud)
//!
//! # Log format
//!   Directory  : D000001/ (shared with imu_data / baro_data, one per boot)
//!   File name  : D000001/000001.MAG, 000002.MAG, ... (rotates every 30 s)
//!   Each record: COBS-encoded postcard blob, zero-byte terminated.
//!   Struct     : { timestamp_us: u64, x: i16, y: i16, z: i16 }
//!   Sensitivity: 0.3 uT/LSB
//!
//! # Decoding on macOS / Linux
//!   cat /Volumes/<card>/D000001/*.MAG > session.bin
//!
//!   Python:
//!     import struct
//!     from cobs import cobs
//!     from pathlib import Path
//!
//!     def read_varint(d, p):
//!         r, s = 0, 0
//!         while True:
//!             b = d[p]; p += 1
//!             r |= (b & 0x7F) << s
//!             if not (b & 0x80): return r, p
//!             s += 7
//!
//!     def read_zigzag_i16(d, p):
//!         v, p = read_varint(d, p)
//!         return (v >> 1) ^ -(v & 1), p
//!
//!     raw = Path("session.bin").read_bytes()
//!     for pkt in raw.split(b'\x00'):
//!         if not pkt: continue
//!         dec = cobs.decode(pkt)
//!         ts, pos = read_varint(dec, 0)
//!         x, pos = read_zigzag_i16(dec, pos)
//!         y, pos = read_zigzag_i16(dec, pos)
//!         z, pos = read_zigzag_i16(dec, pos)
//!         print(f"t={ts/1e6:.3f}s  mx={x*0.3:.1f}uT  my={y*0.3:.1f}uT  mz={z*0.3:.1f}uT")

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::{String, Vec};
use postcard::to_slice_cobs;
use serde::Serialize;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::qmc5883l::{Qmc5883l, ADDR as QMC_ADDR};
use micoairh743v2::resources::I2c2Irqs;
use micoairh743v2::sdlog::SdmmcResources;

// ── Log record ───────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct MagSample {
    timestamp_us: u64,
    x: i16,
    y: i16,
    z: i16,
}

// ── Interrupt bindings ───────────────────────────────────────────────────────

// DMA1_STREAM1 (TIM1 UP DMA for ESC silence) is bound at lib level (resources::MotorIrqs).
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

// ── Entry point ──────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    // ── UART ────────────────────────────────────────────────────────────────
    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default()).unwrap();
    uart.write(b"mag_log: UART ok\r\n").await.ok();

    // ── ESC silence ──────────────────────────────────────────────────────────
    // Spawn a task that sends DShot disarm frames continuously so ESCs stay
    // quiet for the lifetime of this binary.
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());
    uart.write(b"mag_log: ESC silence task started\r\n").await.ok();

    // ── I2C2 / IST8310 ──────────────────────────────────────────────────────
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;

    let mut i2c = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, I2c2Irqs, i2c_cfg);

    // Probe address before driver init to distinguish "device absent" from
    // "device present but init failed".
    if i2c.write(QMC_ADDR, &[]).await.is_ok() {
        uart.write(b"mag_log: QMC5883L found at 0x0D\r\n").await.ok();
    } else {
        uart.write(b"mag_log: NOTHING at 0x0D -- check wiring\r\n").await.ok();
    }

    let mut mag = Qmc5883l::new(i2c);

    match mag.init().await {
        Ok(()) => {
            uart.write(b"mag_log: QMC5883L ok\r\n").await.ok();
        }
        Err(_) => {
            uart.write(b"mag_log: QMC5883L init err\r\n").await.ok();
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

    let mut sd_ok = false;
    for attempt in 1u8..=5 {
        match device.try_reset().await {
            Ok(()) => { sd_ok = true; break; }
            Err(e) => {
                let reason = match e {
                    embassy_stm32::sdmmc::Error::NoCard              => "no card inserted",
                    embassy_stm32::sdmmc::Error::Timeout             => "card not responding (inserted?)",
                    embassy_stm32::sdmmc::Error::SoftwareTimeout     => "software timeout",
                    embassy_stm32::sdmmc::Error::Crc                 => "CRC error (loose connection?)",
                    embassy_stm32::sdmmc::Error::UnsupportedCardVersion => "unsupported card version",
                    embassy_stm32::sdmmc::Error::UnsupportedCardType => "unsupported card type",
                    embassy_stm32::sdmmc::Error::UnsupportedVoltage  => "unsupported voltage",
                    _                                                 => "hardware error",
                };
                let mut s: String<64> = String::new();
                write!(s, "mag_log: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"mag_log: SD FAIL (giving up)\r\n").await.ok();
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
    uart.write(b"mag_log: SD ok\r\n").await.ok();

    // ── FAT filesystem ───────────────────────────────────────────────────────
    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(e) => {
            let reason = match e {
                embedded_fatfs::Error::CorruptedFileSystem => "corrupted filesystem (reformat?)",
                embedded_fatfs::Error::Io(_)              => "I/O error reading FAT structures",
                _                                         => "unexpected FAT error",
            };
            let mut s: String<64> = String::new();
            write!(s, "mag_log: FAT mount FAIL: {}\r\n", reason).ok();
            uart.write(s.as_bytes()).await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };
    uart.write(b"mag_log: FAT ok\r\n").await.ok();

    // ── Session directory ────────────────────────────────────────────────────
    // Reuse the highest D%06u directory if one exists (created by imu_data or
    // baro_data on this boot) so all sensor logs share the same folder.
    let mut session_idx: u32 = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        if entry.is_dir() {
            if let Some(idx) = parse_session_dir_idx(entry.short_file_name_as_bytes()) {
                session_idx = session_idx.max(idx);
            }
        }
    }
    if session_idx == 0 {
        session_idx = 1;
    }

    let mut dir_name: String<8> = String::new();
    write!(dir_name, "D{:06}", session_idx).ok();

    let mut msg: String<32> = String::new();
    write!(msg, "mag_log: session {}\r\n", dir_name.as_str()).ok();
    uart.write(msg.as_bytes()).await.ok();

    let session_dir = match fs.root_dir().create_dir(dir_name.as_str()).await {
        Ok(d) => d,
        Err(embedded_fatfs::Error::AlreadyExists) => fs.root_dir().open_dir(dir_name.as_str()).await.unwrap_or_else(|_| loop {}),
        Err(e) => {
            let reason = match e {
                embedded_fatfs::Error::NotEnoughSpace      => "disk full",
                embedded_fatfs::Error::CorruptedFileSystem => "corrupted filesystem",
                _                                          => "unexpected error",
            };
            let mut s: String<64> = String::new();
            write!(s, "mag_log: session dir FAIL: {}\r\n", reason).ok();
            uart.write(s.as_bytes()).await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    let mut file_idx: u16 = 1;
    let mut fname: String<12> = String::new();
    write!(fname, "{:06}.MAG", file_idx).ok();

    let mut file = match session_dir.create_file(fname.as_str()).await {
        Ok(f) => {
            let mut m: String<48> = String::new();
            write!(m, "mag_log: {}/{} open ok\r\n", dir_name.as_str(), fname.as_str()).ok();
            uart.write(m.as_bytes()).await.ok();
            f
        }
        Err(e) => {
            let reason = match e {
                embedded_fatfs::Error::NotEnoughSpace      => "disk full",
                embedded_fatfs::Error::AlreadyExists       => "file already exists",
                embedded_fatfs::Error::CorruptedFileSystem => "corrupted filesystem",
                _                                          => "unexpected error",
            };
            let mut s: String<64> = String::new();
            write!(s, "mag_log: file create FAIL: {}\r\n", reason).ok();
            uart.write(s.as_bytes()).await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    uart.write(b"mag_log: logging started\r\n").await.ok();
    led_green.set_high();

    // ── Logging loop ─────────────────────────────────────────────────────────
    // Worst-case postcard size for MagSample: 10 (u64 varint) + 3x3 (i16 zigzag) = 19 B.
    // COBS overhead: 1 B. Sentinel: 1 B. Total <= 21 B. Use 24 bytes.
    let mut serde_buf = [0u8; 24];
    let mut write_buf: Vec<u8, 512> = Vec::new();
    let mut last_flush  = Instant::now();
    let mut last_rotate = Instant::now();
    let mut mag_bytes_total: u32 = 0;

    const ROTATE_SECS: u64 = 30;

    loop {
        match mag.read().await {
            Ok(d) => {
                let sample = MagSample {
                    timestamp_us: Instant::now().as_micros(),
                    x: d.x,
                    y: d.y,
                    z: d.z,
                };

                // UART live reading (uT, 0.3 uT/LSB).
                let ux = (d.x as i32 * 3) / 10;
                let uy = (d.y as i32 * 3) / 10;
                let uz = (d.z as i32 * 3) / 10;
                let mut s: String<64> = String::new();
                write!(s, "mx={}uT my={}uT mz={}uT\r\n", ux, uy, uz).ok();
                uart.write(s.as_bytes()).await.ok();

                if let Ok(encoded) = to_slice_cobs(&sample, &mut serde_buf) {
                    if write_buf.extend_from_slice(encoded).is_err() {
                        led_blue.set_high();
                        if file.write_all(&write_buf).await.is_ok() {
                            mag_bytes_total += write_buf.len() as u32;
                        }
                        write_buf.clear();
                        led_blue.set_low();
                        write_buf.extend_from_slice(encoded).ok();
                    }
                }
            }
            Err(_) => {
                uart.write(b"mag_log: read err\r\n").await.ok();
                led_red.set_high();
                Timer::after_millis(100).await;
                led_red.set_low();
            }
        }

        // Timer-driven flush.
        if last_flush.elapsed() > Duration::from_millis(500) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    mag_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            file.flush().await.ok();
            led_blue.set_low();
            last_flush = Instant::now();
        }

        // File rotation.
        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    mag_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            drop(file);

            let mut m: String<48> = String::new();
            write!(m, "mag_log: closed {} ({} B)\r\n", fname.as_str(), mag_bytes_total).ok();
            uart.write(m.as_bytes()).await.ok();

            file_idx = file_idx.wrapping_add(1);
            fname.clear();
            write!(fname, "{:06}.MAG", file_idx).ok();

            file = match session_dir.create_file(fname.as_str()).await {
                Ok(f) => f,
                Err(e) => {
                    let reason = match e {
                        embedded_fatfs::Error::NotEnoughSpace => "disk full",
                        _                                     => "unexpected error",
                    };
                    let mut s: String<64> = String::new();
                    write!(s, "mag_log: rotate FAIL: {}\r\n", reason).ok();
                    uart.write(s.as_bytes()).await.ok();
                    loop { Timer::after_secs(1).await; }
                }
            };
            led_blue.set_low();

            let mut msg2: String<48> = String::new();
            write!(msg2, "mag_log: -> {}\r\n", fname.as_str()).ok();
            uart.write(msg2.as_bytes()).await.ok();

            last_rotate = Instant::now();
            last_flush  = Instant::now();
        }

        Timer::after_millis(100).await;
    }
}

/// Parse session index from a `D%06u` FAT 8.3 short name.
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
