//! MicoAir H743v2 -- GPS UBX data reader + SD logger.
//!
//! Reads UBX binary frames from the MicoAir MG-A01 GPS module on
//! USART3, prints parsed NAV-PVT / NAV-SAT to USART1 (microterm),
//! and writes COBS+postcard GPS records to SDMMC1 for offline use.
//!
//! Each run creates a new session directory (D%06u) on the SD card
//! with a single .GPS file containing serialized GpsRecord structs.
//!
//! # Hardware
//!   GPS    USART3: RX=PD9  (TX=PD8 unused)  115200 baud
//!   UART1: TX=PA9  115200 baud (debug output)
//!   SDMMC1: CLK=PC12  CMD=PD2  D0-D3=PC8-PC11
//!
//! # LEDs
//!   Green : steady after init
//!   Blue  : toggles on SD write
//!   Red   : rapid blink on SD init failure

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartRx, UartTx};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::{String, Vec};
use postcard::to_slice_cobs;
use serde::Serialize;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::resources::{SensorIrqs, UartLogIrqs};
use micoairh743v2::sdlog::SdmmcResources;

const GPS_BAUD: u32 = 115_200;

// UBX protocol constants
const UBX_SYNC_1: u8 = 0xB5;
const UBX_SYNC_2: u8 = 0x62;
const NAV_CLASS: u8 = 0x01;
const NAV_PVT_ID: u8 = 0x07;
const NAV_PVT_LEN: usize = 92;
const NAV_SAT_ID: u8 = 0x35;
// NAV-SAT: 8-byte header + 12 bytes per satellite. 512 covers ~42 sats.
const MAX_PAYLOAD: usize = 512;

const FLUSH_PERIOD: Duration = Duration::from_millis(500);

// ── Log record ──────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct GpsRecord {
    timestamp_us: u64,
    fix_type: u8,
    num_sv: u8,
    lon_1e7: i32,
    lat_1e7: i32,
    h_msl_mm: i32,
    g_speed_mm_s: i32,
    head_mot_1e5: i32,
    h_acc_mm: u32,
    v_acc_mm: u32,
}

// ── Helpers ─────────────────────────────────────────────────────────────────

fn le_i32(buf: &[u8], off: usize) -> i32 {
    i32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn le_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn fix_type_str(ft: u8) -> &'static str {
    match ft {
        0 => "none",
        1 => "DR",
        2 => "2D",
        3 => "3D",
        4 => "3D+DR",
        5 => "time",
        _ => "?",
    }
}

// ── Entry point ─────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PE3, Level::Low, Speed::Low);

    // ── Debug UART ───────────────────────────────────────────────────────────
    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default()).unwrap();
    uart.write(b"gps_data: UART1 TX ok\r\n").await.ok();

    // ── GPS UART (USART3 RX only) ────────────────────────────────────────────
    let mut gps_cfg = UartConfig::default();
    gps_cfg.baudrate = GPS_BAUD;
    let mut gps_rx = UartRx::new(p.USART3, p.PD9, p.DMA1_CH2, SensorIrqs, gps_cfg).unwrap();
    uart.write(b"gps_data: USART3 RX ok\r\n").await.ok();

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
            Ok(()) => {
                sd_ok = true;
                break;
            }
            Err(e) => {
                let reason = match e {
                    embassy_stm32::sdmmc::Error::NoCard => "no card",
                    embassy_stm32::sdmmc::Error::Timeout => "timeout",
                    embassy_stm32::sdmmc::Error::SoftwareTimeout => "sw timeout",
                    embassy_stm32::sdmmc::Error::Crc => "CRC",
                    _ => "hw error",
                };
                let mut s: String<64> = String::new();
                write!(s, "gps_data: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"gps_data: SD FAIL\r\n").await.ok();
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

    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"gps_data: FAT mount FAIL\r\n").await.ok();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    uart.write(b"gps_data: SD ok\r\n").await.ok();

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
    session_idx += 1;

    let mut dir_name: String<8> = String::new();
    let session_dir = loop {
        dir_name.clear();
        write!(dir_name, "D{:06}", session_idx).ok();
        match fs.root_dir().create_dir(dir_name.as_str()).await {
            Ok(d) => break d,
            Err(embedded_fatfs::Error::AlreadyExists) => {
                session_idx += 1;
                continue;
            }
            Err(_) => {
                uart.write(b"gps_data: session dir FAIL\r\n").await.ok();
                loop {
                    Timer::after_secs(1).await;
                }
            }
        }
    };
    let mut session_msg: String<32> = String::new();
    write!(session_msg, "gps_data: session {}\r\n", dir_name.as_str()).ok();
    uart.write(session_msg.as_bytes()).await.ok();

    // ── Open log file ────────────────────────────────────────────────────────
    let mut file = match session_dir.create_file("000001.GPS").await {
        Ok(f) => f,
        Err(_) => {
            uart.write(b"gps_data: file create FAIL\r\n").await.ok();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    uart.write(b"gps_data: logging started\r\n").await.ok();
    led_green.set_high();

    // ── Write buffer ─────────────────────────────────────────────────────────
    let mut buf_sd: Vec<u8, 512> = Vec::new();
    let mut serde_tmp = [0u8; 64];
    let mut last_flush = Instant::now();

    // ── UBX read buffers ─────────────────────────────────────────────────────
    let mut b = [0u8; 1];
    let mut hdr = [0u8; 4];
    let mut pbuf = [0u8; MAX_PAYLOAD + 2];

    // ── Main loop ────────────────────────────────────────────────────────────
    loop {
        // Sync to UBX preamble: 0xB5 0x62
        loop {
            gps_rx.read(&mut b).await.ok();
            if b[0] != UBX_SYNC_1 {
                continue;
            }
            gps_rx.read(&mut b).await.ok();
            if b[0] == UBX_SYNC_2 {
                break;
            }
        }

        // Read class + ID + length
        gps_rx.read(&mut hdr).await.ok();
        let class = hdr[0];
        let id = hdr[1];
        let len = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;

        if class != NAV_CLASS || len > MAX_PAYLOAD {
            continue;
        }

        // Read payload + 2 checksum bytes
        gps_rx.read(&mut pbuf[..len + 2]).await.ok();

        let now = Instant::now();

        match id {
            NAV_PVT_ID if len == NAV_PVT_LEN => {
                let rec = GpsRecord {
                    timestamp_us: now.as_micros(),
                    fix_type: pbuf[20],
                    num_sv: pbuf[23],
                    lon_1e7: le_i32(&pbuf, 24),
                    lat_1e7: le_i32(&pbuf, 28),
                    h_msl_mm: le_i32(&pbuf, 36),
                    g_speed_mm_s: le_i32(&pbuf, 60),
                    head_mot_1e5: le_i32(&pbuf, 64),
                    h_acc_mm: le_u32(&pbuf, 40),
                    v_acc_mm: le_u32(&pbuf, 44),
                };

                // Write to SD
                if let Ok(encoded) = to_slice_cobs(&rec, &mut serde_tmp) {
                    if buf_sd.extend_from_slice(encoded).is_err() {
                        led_blue.set_high();
                        file.write_all(&buf_sd).await.ok();
                        buf_sd.clear();
                        led_blue.set_low();
                        buf_sd.extend_from_slice(encoded).ok();
                    }
                }

                // Print to UART
                print_nav_pvt(&pbuf, &mut uart).await;
            }
            NAV_SAT_ID if len >= 8 => {
                print_nav_sat(&pbuf[..len], &mut uart).await;
            }
            _ => {}
        }

        // Periodic flush
        if last_flush.elapsed() > FLUSH_PERIOD {
            if !buf_sd.is_empty() {
                led_blue.set_high();
                file.write_all(&buf_sd).await.ok();
                buf_sd.clear();
                led_blue.set_low();
            }
            file.flush().await.ok();
            last_flush = Instant::now();
        }
    }
}

// ── UBX printers ────────────────────────────────────────────────────────────

async fn print_nav_pvt(p: &[u8], uart: &mut UartTx<'_, embassy_stm32::mode::Async>) {
    let fix = p[20];
    let sats = p[23];
    let lon = le_i32(p, 24);
    let lat = le_i32(p, 28);
    let alt = le_i32(p, 36);
    let speed = le_i32(p, 60);

    let lat_deg = lat / 10_000_000;
    let lat_frac = (lat % 10_000_000).unsigned_abs();
    let lon_deg = lon / 10_000_000;
    let lon_frac = (lon % 10_000_000).unsigned_abs();
    let alt_m = alt / 1000;
    let alt_dm = ((alt % 1000).unsigned_abs()) / 100;
    let spd_ms = speed / 1000;
    let spd_cm = ((speed % 1000).unsigned_abs()) / 10;

    let mut msg: String<128> = String::new();
    write!(
        msg,
        "fix={} sat={:2} lat={}.{:07} lon={}.{:07} alt={}.{}m spd={}.{:02}m/s\r\n",
        fix_type_str(fix),
        sats,
        lat_deg,
        lat_frac,
        lon_deg,
        lon_frac,
        alt_m,
        alt_dm,
        spd_ms,
        spd_cm,
    )
    .ok();
    uart.write(msg.as_bytes()).await.ok();
}

async fn print_nav_sat(p: &[u8], uart: &mut UartTx<'_, embassy_stm32::mode::Async>) {
    let num_svs = p[5] as usize;

    // Count tracked and used satellites per constellation.
    // Indices: 0=GPS 1=SBAS 2=GAL 3=BDS 4=QZSS 5=GLO 6=other
    let mut tracked = [0u8; 7];
    let mut used = [0u8; 7];

    for i in 0..num_svs {
        let off = 8 + i * 12;
        if off + 12 > p.len() {
            break;
        }

        let gnss_id = p[off];
        let flags = u32::from_le_bytes([p[off + 8], p[off + 9], p[off + 10], p[off + 11]]);
        let sv_used = (flags >> 3) & 1 == 1;

        let idx = match gnss_id {
            0 => 0, // GPS
            1 => 1, // SBAS
            2 => 2, // Galileo
            3 => 3, // BeiDou
            5 => 4, // QZSS
            6 => 5, // GLONASS
            _ => 6,
        };
        tracked[idx] += 1;
        if sv_used {
            used[idx] += 1;
        }
    }

    let ids: [(usize, &str); 6] = [
        (0, "GPS"),
        (5, "GLO"),
        (2, "GAL"),
        (3, "BDS"),
        (1, "SBAS"),
        (4, "QZSS"),
    ];

    let mut msg: String<128> = String::new();
    write!(msg, "SAT:").ok();
    for (idx, name) in ids {
        if tracked[idx] > 0 {
            write!(msg, " {}={}/{}", name, used[idx], tracked[idx]).ok();
        }
    }
    write!(msg, "\r\n").ok();
    uart.write(msg.as_bytes()).await.ok();
}

// ── Utility ─────────────────────────────────────────────────────────────────

fn parse_session_dir_idx(name: &[u8]) -> Option<u32> {
    if name.len() != 7 || (name[0] != b'D' && name[0] != b'd') {
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
