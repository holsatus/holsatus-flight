//! MicoAir H743v2 -- Battery voltage and current monitor with SD logging.
//!
//! Samples ADC1 at 10 Hz, prints voltage + raw current to UART1, and writes
//! COBS+postcard records to a `.BAT` file on SDMMC1.
//!
//! # Hardware
//!   PC0: battery voltage via 1:21 divider (V_bat = V_adc * 21)
//!   PC1: current sensor (raw mV; calibration depends on sensor type)
//!   SDMMC1: CLK=PC12  CMD=PD2  D0-D3=PC8-PC11
//!   UART1:  TX=PA9  (115200 baud)
//!
//! # USB power (no LiPo):
//!   Voltage: reads ~0 V (divider top rail is 0 V)
//!   Current: reads ~0 mV (shunt sensor) or ~1650 mV (Hall sensor zero-current midpoint)
//!
//! # Voltage formula
//!   V_bat = (raw / 65535) * 3300 mV * 21
//!
//! # Current sensor
//!   Raw millivolts are printed to help identify the sensor type:
//!     ~0 mV at 0 A       -> shunt-based (uni-directional)
//!     ~1650 mV at 0 A    -> Hall/bi-directional (e.g. ACS712)
//!   Once identified, apply the sensor's mV/A scale factor.
//!
//! # Log format
//!   Directory : D000001/ (highest existing session dir, or new if none)
//!   File name : D000001/000001.BAT, 000002.BAT, ... (rotates every 30 s)
//!   Each record: COBS-encoded postcard blob, zero-byte terminated.
//!   Struct    : { timestamp_us: u64, voltage_mv: u32, current_mv: u32 }

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::{String, Vec};
use postcard::to_slice_cobs;
use serde::Serialize;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::sdlog::SdmmcResources;

// ── Log record ───────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct BatSample {
    timestamp_us: u64,
    voltage_mv:   u32,
    current_mv:   u32,
}

// ── Interrupt bindings ───────────────────────────────────────────────────────

// DMA1_STREAM1 (TIM1 UP DMA) is bound at lib level (resources::MotorIrqs).
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

// Voltage divider ratio.
const V_DIV: u32 = 21;
// ADC full-scale counts (16-bit H7 ADC).
const ADC_FULL: u32 = 65535;
// Reference voltage in millivolts.
const VREF_MV: u32 = 3300;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default()).unwrap();
    uart.write(b"battery_data: UART ok\r\n").await.ok();

    // ── ESC silence ──────────────────────────────────────────────────────────
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());

    // ── ADC1 (blocking reads, no DMA needed at 10 Hz) ────────────────────────
    let mut adc = Adc::new(p.ADC1);
    let mut pin_v = p.PC0;
    let mut pin_i = p.PC1;
    uart.write(b"battery_data: ADC ok\r\n").await.ok();

    // ── SDMMC1 ───────────────────────────────────────────────────────────────
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
                write!(s, "battery_data: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"battery_data: SD FAIL (continuing without SD)\r\n").await.ok();
        // Run without SD -- fall through to sampling loop, no file writes.
        led_green.set_high();
        uart.write(b"battery_data: sampling at 10 Hz (UART only)\r\n").await.ok();
        loop {
            let raw_v = adc.blocking_read(&mut pin_v, SampleTime::CYCLES64_5);
            let raw_i = adc.blocking_read(&mut pin_i, SampleTime::CYCLES64_5);
            let v_bat_mv = (raw_v as u32 * VREF_MV * V_DIV) / ADC_FULL;
            let i_raw_mv = (raw_i as u32 * VREF_MV) / ADC_FULL;
            let mut s: String<64> = String::new();
            write!(s, "V_bat={} mV  I_raw={} mV\r\n", v_bat_mv, i_raw_mv).ok();
            uart.write(s.as_bytes()).await.ok();
            if v_bat_mv > 5000 { led_red.set_high(); } else { led_red.set_low(); }
            Timer::after_millis(100).await;
        }
    }
    uart.write(b"battery_data: SD ok\r\n").await.ok();

    // ── FAT filesystem ───────────────────────────────────────────────────────
    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"battery_data: FAT mount FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };
    uart.write(b"battery_data: FAT ok\r\n").await.ok();

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
    write!(msg, "battery_data: session {}\r\n", dir_name.as_str()).ok();
    uart.write(msg.as_bytes()).await.ok();

    let session_dir = match fs.root_dir().create_dir(dir_name.as_str()).await {
        Ok(d) => d,
        Err(embedded_fatfs::Error::AlreadyExists) => {
            fs.root_dir().open_dir(dir_name.as_str()).await.unwrap_or_else(|_| loop {})
        }
        Err(_) => {
            uart.write(b"battery_data: session dir FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    let mut file_idx: u16 = 1;
    let mut fname: String<12> = String::new();
    write!(fname, "{:06}.BAT", file_idx).ok();

    let mut file = match session_dir.create_file(fname.as_str()).await {
        Ok(f) => {
            let mut m: String<48> = String::new();
            write!(m, "battery_data: {}/{} open ok\r\n", dir_name.as_str(), fname.as_str()).ok();
            uart.write(m.as_bytes()).await.ok();
            f
        }
        Err(_) => {
            uart.write(b"battery_data: file create FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };

    uart.write(b"battery_data: logging at 10 Hz\r\n").await.ok();
    uart.write(b"battery_data: V_bat[mV]  I_raw[mV]  (0A: ~0 shunt, ~1650 Hall)\r\n").await.ok();
    led_green.set_high();

    // ── Logging loop ─────────────────────────────────────────────────────────
    let mut serde_buf = [0u8; 24];
    let mut write_buf: Vec<u8, 512> = Vec::new();
    let mut last_flush  = Instant::now();
    let mut last_rotate = Instant::now();
    let mut bat_bytes_total: u32 = 0;

    const ROTATE_SECS: u64 = 30;

    loop {
        let raw_v = adc.blocking_read(&mut pin_v, SampleTime::CYCLES64_5);
        let raw_i = adc.blocking_read(&mut pin_i, SampleTime::CYCLES64_5);

        let v_bat_mv = (raw_v as u32 * VREF_MV * V_DIV) / ADC_FULL;
        let i_raw_mv = (raw_i as u32 * VREF_MV) / ADC_FULL;

        let mut s: String<64> = String::new();
        write!(s, "V_bat={} mV  I_raw={} mV\r\n", v_bat_mv, i_raw_mv).ok();
        uart.write(s.as_bytes()).await.ok();

        if v_bat_mv > 5000 { led_red.set_high(); } else { led_red.set_low(); }

        let sample = BatSample {
            timestamp_us: Instant::now().as_micros(),
            voltage_mv:   v_bat_mv,
            current_mv:   i_raw_mv,
        };

        if let Ok(encoded) = to_slice_cobs(&sample, &mut serde_buf) {
            if write_buf.extend_from_slice(encoded).is_err() {
                led_blue.set_high();
                if file.write_all(&write_buf).await.is_ok() {
                    bat_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
                led_blue.set_low();
                write_buf.extend_from_slice(encoded).ok();
            }
        }

        if last_flush.elapsed() > Duration::from_millis(500) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    bat_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            file.flush().await.ok();
            led_blue.set_low();
            last_flush = Instant::now();
        }

        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();
            if !write_buf.is_empty() {
                if file.write_all(&write_buf).await.is_ok() {
                    bat_bytes_total += write_buf.len() as u32;
                }
                write_buf.clear();
            }
            drop(file);

            let mut m: String<48> = String::new();
            write!(m, "battery_data: closed {} ({} B)\r\n", fname.as_str(), bat_bytes_total).ok();
            uart.write(m.as_bytes()).await.ok();
            bat_bytes_total = 0;

            file_idx = file_idx.wrapping_add(1);
            fname.clear();
            write!(fname, "{:06}.BAT", file_idx).ok();

            file = match session_dir.create_file(fname.as_str()).await {
                Ok(f) => f,
                Err(_) => {
                    uart.write(b"battery_data: rotate FAIL\r\n").await.ok();
                    loop { Timer::after_secs(1).await; }
                }
            };
            led_blue.set_low();

            let mut msg2: String<48> = String::new();
            write!(msg2, "battery_data: -> {}\r\n", fname.as_str()).ok();
            uart.write(msg2.as_bytes()).await.ok();

            last_rotate = Instant::now();
            last_flush  = Instant::now();
        }

        Timer::after_millis(100).await;
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
