//! MicoAir H743v2 -- BMI270 IMU data logger to SD card.
//!
//! Reads BMI270 at 1 kHz over SPI3 and writes COBS+postcard records to a
//! sequentially-named `.B70` file on SDMMC1. UART1 prints a live summary
//! at 20 Hz.
//!
//! # Hardware
//!   BMI270 SPI3: SCLK=PB3  MOSI=PD6  MISO=PB4  CS=PA15  DR=PB7
//!   SDMMC1:      CLK=PC12  CMD=PD2   D0-D3=PC8-PC11
//!   UART1:       TX=PA9  (115 200 baud)
//!
//! # Log format
//!   Directory  : D000001/ (shared with other sensor logs, one per boot)
//!   File name  : D000001/000001.B70, 000002.B70, ... (rotates every 30 s)
//!   Each record: COBS-encoded postcard blob, zero-byte terminated.
//!   Struct     : { timestamp_us: u64, acc: [i16; 3], gyr: [i16; 3] }
//!   Accel scale: +-2 g range  -> 1 g = 16384 LSB
//!   Gyro  scale: +-2000 dps   -> 1 dps = 16.384 LSB
//!
//! # Decoding on macOS / Linux
//!   cat /Volumes/<card>/D000001/*.B70 > session.bin
//!   (same decoder as imu_data.rs)
//!
//! WARNING: BMI270 init takes ~200 ms (config blob upload).

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

use micoairh743v2::bmi270::{Bmi270, BMI270_CONFIG_FILE};
use micoairh743v2::resources::Spi3Irqs;
use micoairh743v2::sdlog::SdmmcResources;

// ── Log record ───────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct ImuSample {
    timestamp_us: u64,
    acc: [i16; 3],
    gyr: [i16; 3],
}

// ── Interrupt bindings ───────────────────────────────────────────────────────

// SPI3 DMA (DMA2_CH0 RX, DMA2_CH1 TX) is bound at lib level (resources::Spi3Irqs).
// DMA1_STREAM1 (TIM1 UP DMA) is bound at lib level (resources::MotorIrqs).
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

type Spi3Bus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
static SPI3_BUS: StaticCell<Spi3Bus> = StaticCell::new();

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
    uart.write(b"bmi270_log: UART ok\r\n").await.ok();

    // ── ESC silence ──────────────────────────────────────────────────────────
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());
    uart.write(b"bmi270_log: ESC silence task started\r\n").await.ok();

    // ── SPI3 / BMI270 ────────────────────────────────────────────────────────
    let cs = Output::new(p.PA15, Level::High, Speed::High);

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(1_000_000); // conservative during init
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;

    let spi = Spi::new(
        p.SPI3, p.PB3, p.PD6, p.PB4,
        p.DMA2_CH1, p.DMA2_CH0,
        Spi3Irqs,
        spi_cfg,
    );
    let bus: &'static Spi3Bus = SPI3_BUS.init(Mutex::new(spi));

    let dev_cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(10_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = Pull::Up;
        c
    };
    let dev = SpiDeviceWithConfig::new(bus, cs, dev_cfg);
    let mut imu = Bmi270::new(dev);

    uart.write(b"bmi270_log: BMI270 init (uploading config blob ~200 ms)...\r\n").await.ok();
    match imu.init(&BMI270_CONFIG_FILE).await {
        Ok(()) => {
            uart.write(b"bmi270_log: BMI270 ok\r\n").await.ok();
        }
        Err(e) => {
            let mut s: String<64> = String::new();
            match e {
                micoairh743v2::bmi270::Error::Spi             => s.push_str("bmi270_log: BMI270 SPI err\r\n").ok(),
                micoairh743v2::bmi270::Error::WrongChipId(id) => write!(s, "bmi270_log: BMI270 wrong id=0x{:02X}\r\n", id).ok(),
                micoairh743v2::bmi270::Error::InitFailed { status } => write!(s, "bmi270_log: BMI270 init failed status=0x{:02X}\r\n", status).ok(),
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

    let mut sd_ok = false;
    for attempt in 1u8..=5 {
        match device.try_reset().await {
            Ok(()) => { sd_ok = true; break; }
            Err(e) => {
                let reason = match e {
                    embassy_stm32::sdmmc::Error::NoCard          => "no card inserted",
                    embassy_stm32::sdmmc::Error::Timeout         => "card not responding",
                    embassy_stm32::sdmmc::Error::SoftwareTimeout => "software timeout",
                    embassy_stm32::sdmmc::Error::Crc             => "CRC error",
                    _                                             => "hardware error",
                };
                let mut s: String<64> = String::new();
                write!(s, "bmi270_log: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"bmi270_log: SD FAIL (giving up)\r\n").await.ok();
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
    uart.write(b"bmi270_log: SD ok\r\n").await.ok();

    // ── FAT filesystem ───────────────────────────────────────────────────────
    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"bmi270_log: FAT mount FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };
    uart.write(b"bmi270_log: FAT ok\r\n").await.ok();

    // ── Session directory ────────────────────────────────────────────────────
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
    write!(msg, "bmi270_log: session {}\r\n", dir_name.as_str()).ok();
    uart.write(msg.as_bytes()).await.ok();

    let session_dir = match fs.root_dir().create_dir(dir_name.as_str()).await {
        Ok(d) => d,
        Err(embedded_fatfs::Error::AlreadyExists) => {
            fs.root_dir().open_dir(dir_name.as_str()).await.unwrap_or_else(|_| loop {})
        }
        Err(_) => {
            uart.write(b"bmi270_log: session dir FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    let mut file_idx: u16 = 1;
    let mut fname: String<12> = String::new();
    write!(fname, "{:06}.B70", file_idx).ok();

    let mut file = match session_dir.create_file(fname.as_str()).await {
        Ok(f) => {
            let mut m: String<48> = String::new();
            write!(m, "bmi270_log: {}/{} open ok\r\n", dir_name.as_str(), fname.as_str()).ok();
            uart.write(m.as_bytes()).await.ok();
            f
        }
        Err(_) => {
            uart.write(b"bmi270_log: file create FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    uart.write(b"bmi270_log: logging started\r\n").await.ok();
    led_green.set_high();

    // ── Logging loop ─────────────────────────────────────────────────────────
    let mut serde_buf = [0u8; 30];
    let mut write_buf: Vec<u8, 512> = Vec::new();
    let mut last_flush  = Instant::now();
    let mut last_rotate = Instant::now();
    let mut b70_bytes_total: u32 = 0;
    let mut print_divider: u8 = 0;

    const ROTATE_SECS: u64 = 30;

    let mut ticker = Ticker::every(Duration::from_hz(1000));

    loop {
        ticker.next().await;

        match imu.read().await {
            Ok(d) => {
                let sample = ImuSample {
                    timestamp_us: Instant::now().as_micros(),
                    acc: [d.accel.x, d.accel.y, d.accel.z],
                    gyr: [d.gyro.x,  d.gyro.y,  d.gyro.z],
                };

                // Live UART at 20 Hz.
                print_divider = print_divider.wrapping_add(1);
                if print_divider >= 50 {
                    print_divider = 0;
                    let mut s: String<128> = String::new();
                    write!(
                        s,
                        "ax={} ay={} az={}  gx={} gy={} gz={}\r\n",
                        d.accel.x, d.accel.y, d.accel.z,
                        d.gyro.x,  d.gyro.y,  d.gyro.z,
                    ).ok();
                    uart.write(s.as_bytes()).await.ok();
                }

                if let Ok(encoded) = to_slice_cobs(&sample, &mut serde_buf) {
                    if write_buf.extend_from_slice(encoded).is_err() {
                        led_blue.set_high();
                        if file.write_all(&write_buf).await.is_ok() {
                            b70_bytes_total += write_buf.len() as u32;
                        }
                        write_buf.clear();
                        led_blue.set_low();
                        write_buf.extend_from_slice(encoded).ok();
                    }
                }
            }
            Err(_) => {
                uart.write(b"bmi270_log: read err\r\n").await.ok();
                led_red.set_high();
                Timer::after_millis(10).await;
                led_red.set_low();
            }
        }

        // Timer-driven flush every 500 ms.
        if last_flush.elapsed() > Duration::from_millis(500) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    b70_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            file.flush().await.ok();
            led_blue.set_low();
            last_flush = Instant::now();
        }

        // File rotation every 30 s.
        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    b70_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            drop(file);

            let mut m: String<48> = String::new();
            write!(m, "bmi270_log: closed {} ({} B)\r\n", fname.as_str(), b70_bytes_total).ok();
            uart.write(m.as_bytes()).await.ok();

            file_idx = file_idx.wrapping_add(1);
            fname.clear();
            write!(fname, "{:06}.B70", file_idx).ok();

            file = match session_dir.create_file(fname.as_str()).await {
                Ok(f) => f,
                Err(_) => {
                    uart.write(b"bmi270_log: rotate FAIL\r\n").await.ok();
                    loop { Timer::after_secs(1).await; }
                }
            };
            led_blue.set_low();

            let mut msg2: String<48> = String::new();
            write!(msg2, "bmi270_log: -> {}\r\n", fname.as_str()).ok();
            uart.write(msg2.as_bytes()).await.ok();

            last_rotate = Instant::now();
            last_flush  = Instant::now();
        }
    }
}

/// Parse session index from a `D%06u` FAT 8.3 short name.
fn parse_session_dir_idx(name: &[u8]) -> Option<u32> {
    if name.len() != 7 || (name[0] != b'D' && name[0] != b'd') {
        return None;
    }
    let mut idx: u32 = 0;
    for &b in &name[1..] {
        if b < b'0' || b > b'9' { return None; }
        idx = idx * 10 + (b - b'0') as u32;
    }
    Some(idx)
}
