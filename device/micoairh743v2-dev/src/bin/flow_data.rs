//! MicoAir H743v2 -- MTF-01 optical flow + lidar data logger.
//!
//! Reads MSP v2 frames from the MTF-01 sensor and writes COBS+postcard
//! records to two files per session on SDMMC1. UART1 prints a live summary.
//!
//! # Sensor and rates
//!   MTF-01  USART3  100 Hz  -> D%06u/%06u.OPF  (optical flow, 50 Hz)
//!                           -> D%06u/%06u.LID  (lidar,        50 Hz)
//!
//! # Hardware
//!   MTF-01  USART3: RX=PD9  (TX=PD8 not connected)  115200 baud
//!                   Power: 5V + GND from FC pads
//!   SDMMC1: CLK=PC12  CMD=PD2  D0-D3=PC8-PC11
//!   UART1:  TX=PA9  (115200 baud, debug output)
//!
//! # LEDs
//!   Green : steady after SD and sensor are ready
//!   Blue  : flashes on SD write
//!   Red   : rapid blink on init failure

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

use micoairh743v2::mtf01::{self, Frame, MAX_PAYLOAD};
use micoairh743v2::resources::{SensorIrqs, UartLogIrqs};
use micoairh743v2::sdlog::SdmmcResources;

// This bench rig wires the MTF-01 to USART3 (like the real GPS module) but
// reads it on DMA1_CH3 (like the real MTF-01/UART4 wiring) instead of GPS's
// DMA1_CH2 -- a one-off combination of two otherwise-unrelated vectors,
// which is exactly why `resources::SensorIrqs` bundles GPS + MTF-01 +
// esc_telemetry's UART7 together. See the comment above SensorIrqs in
// resources.rs.

// ── Log records ──────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct FlowRecord {
    timestamp_us: u64,
    quality: u8,
    motion_x: i32,
    motion_y: i32,
}

#[derive(Serialize)]
struct LidRecord {
    timestamp_us: u64,
    quality: u8,
    distance_mm: i32,
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PE3, Level::Low, Speed::Low);

    // ── Debug UART ───────────────────────────────────────────────────────────
    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default()).unwrap();
    uart.write(b"flow_data: UART ok\r\n").await.ok();

    // ── MTF-01 UART (RX only) ────────────────────────────────────────────────
    let mut mtf_cfg = UartConfig::default();
    mtf_cfg.baudrate = 115_200;
    let mut uart_mtf = UartRx::new(p.USART3, p.PD9, p.DMA1_CH3, SensorIrqs, mtf_cfg).unwrap();
    uart.write(b"flow_data: USART3 RX ok\r\n").await.ok();

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
                    embassy_stm32::sdmmc::Error::NoCard => "no card inserted",
                    embassy_stm32::sdmmc::Error::Timeout => "card not responding",
                    embassy_stm32::sdmmc::Error::SoftwareTimeout => "software timeout",
                    embassy_stm32::sdmmc::Error::Crc => "CRC error",
                    _ => "hardware error",
                };
                let mut s: String<64> = String::new();
                write!(s, "flow_data: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"flow_data: SD FAIL\r\n").await.ok();
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
            uart.write(b"flow_data: FAT mount FAIL\r\n").await.ok();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    uart.write(b"flow_data: SD ok\r\n").await.ok();

    // ── Session directory ─────────────────────────────────────────────────────
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
                uart.write(b"flow_data: session dir FAIL\r\n").await.ok();
                loop {
                    Timer::after_secs(1).await;
                }
            }
        }
    };
    let mut session_msg: String<32> = String::new();
    write!(session_msg, "flow_data: session {}\r\n", dir_name.as_str()).ok();
    uart.write(session_msg.as_bytes()).await.ok();

    // ── Open log files ────────────────────────────────────────────────────────
    let mut file_idx: u16 = 1;

    macro_rules! open_file {
        ($ext:literal) => {{
            let mut n: String<12> = String::new();
            write!(n, "{:06}.{}", file_idx, $ext).ok();
            match session_dir.create_file(n.as_str()).await {
                Ok(f) => {
                    let mut m: String<48> = String::new();
                    write!(m, "flow_data: {} open ok\r\n", n.as_str()).ok();
                    uart.write(m.as_bytes()).await.ok();
                    f
                }
                Err(_) => {
                    let mut m: String<48> = String::new();
                    write!(m, "flow_data: {} create FAIL\r\n", n.as_str()).ok();
                    uart.write(m.as_bytes()).await.ok();
                    loop {
                        Timer::after_secs(1).await;
                    }
                }
            }
        }};
    }

    let mut file_opf = open_file!("OPF");
    let mut file_lid = open_file!("LID");

    uart.write(b"flow_data: logging started\r\n").await.ok();
    led_green.set_high();

    // ── Write buffers ─────────────────────────────────────────────────────────
    let mut buf_opf: Vec<u8, 512> = Vec::new();
    let mut buf_lid: Vec<u8, 256> = Vec::new();

    let mut serde_tmp = [0u8; 32];

    let mut bytes_opf: u32 = 0;
    let mut bytes_lid: u32 = 0;

    macro_rules! write_record {
        ($buf:expr, $file:expr, $bytes:expr, $sample:expr) => {{
            if let Ok(encoded) = to_slice_cobs(&$sample, &mut serde_tmp) {
                if $buf.extend_from_slice(encoded).is_err() {
                    led_blue.set_high();
                    if $file.write_all(&$buf).await.is_ok() {
                        $bytes += $buf.len() as u32;
                    }
                    $buf.clear();
                    led_blue.set_low();
                    $buf.extend_from_slice(encoded).ok();
                }
            }
        }};
    }

    // ── Timing ───────────────────────────────────────────────────────────────
    let mut last_flush = Instant::now();
    let mut last_rotate = Instant::now();
    let mut last_print = Instant::now();

    const FLUSH_PERIOD: Duration = Duration::from_millis(500);
    const ROTATE_SECS: u64 = 30;
    const PRINT_PERIOD: Duration = Duration::from_millis(1000);

    // UART read buffers (reused every frame).
    let mut b = [0u8; 1];
    let mut hdr = [0u8; 6];
    let mut pbuf = [0u8; MAX_PAYLOAD + 1];

    // ── Main loop: read MTF-01 frames as they arrive (~100 Hz) ───────────────
    loop {
        // Sync to MSP v2 preamble: $X
        loop {
            uart_mtf.read(&mut b).await.ok();
            if b[0] != b'$' {
                continue;
            }
            uart_mtf.read(&mut b).await.ok();
            if b[0] == b'X' {
                break;
            }
        }

        // Read 6-byte header: dir flags fn_lo fn_hi size_lo size_hi
        uart_mtf.read(&mut hdr).await.ok();

        let size = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if size > MAX_PAYLOAD {
            continue;
        }

        // Read payload + CRC
        uart_mtf.read(&mut pbuf[..size + 1]).await.ok();

        let now = Instant::now();

        match mtf01::parse(hdr, &pbuf[..size + 1]) {
            Some(Frame::Flow(f)) => {
                let rec = FlowRecord {
                    timestamp_us: now.as_micros(),
                    quality: f.quality,
                    motion_x: f.motion_x,
                    motion_y: f.motion_y,
                };
                write_record!(buf_opf, file_opf, bytes_opf, rec);

                if last_print.elapsed() >= PRINT_PERIOD {
                    let mut msg: String<64> = String::new();
                    write!(
                        msg,
                        "OPF q={} x={} y={}\r\n",
                        f.quality, f.motion_x, f.motion_y
                    )
                    .ok();
                    uart.write(msg.as_bytes()).await.ok();
                    last_print = Instant::now();
                }
            }
            Some(Frame::Lidar(l)) => {
                let rec = LidRecord {
                    timestamp_us: now.as_micros(),
                    quality: l.quality,
                    distance_mm: l.distance_mm,
                };
                write_record!(buf_lid, file_lid, bytes_lid, rec);

                if last_print.elapsed() >= PRINT_PERIOD {
                    let mut msg: String<64> = String::new();
                    write!(msg, "LID q={} d={} mm\r\n", l.quality, l.distance_mm).ok();
                    uart.write(msg.as_bytes()).await.ok();
                    last_print = Instant::now();
                }
            }
            None => {} // CRC error or unknown function -- skip
        }

        // ── Periodic flush ────────────────────────────────────────────────────
        if last_flush.elapsed() > FLUSH_PERIOD {
            led_blue.set_high();
            macro_rules! flush_buf {
                ($buf:expr, $file:expr, $bytes:expr) => {
                    if !$buf.is_empty() {
                        if $file.write_all(&$buf).await.is_ok() {
                            $bytes += $buf.len() as u32;
                        }
                        $buf.clear();
                    }
                    $file.flush().await.ok();
                };
            }
            flush_buf!(buf_opf, file_opf, bytes_opf);
            flush_buf!(buf_lid, file_lid, bytes_lid);
            led_blue.set_low();
            last_flush = Instant::now();
        }

        // ── File rotation ─────────────────────────────────────────────────────
        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();
            macro_rules! final_flush {
                ($buf:expr, $file:expr, $bytes:expr) => {
                    if !$buf.is_empty() {
                        if $file.write_all(&$buf).await.is_ok() {
                            $bytes += $buf.len() as u32;
                        }
                        $buf.clear();
                    }
                };
            }
            final_flush!(buf_opf, file_opf, bytes_opf);
            final_flush!(buf_lid, file_lid, bytes_lid);

            let mut rot_msg: String<64> = String::new();
            write!(
                rot_msg,
                "flow_data: rotate {}: OPF={} LID={} B\r\n",
                file_idx, bytes_opf, bytes_lid
            )
            .ok();
            uart.write(rot_msg.as_bytes()).await.ok();

            drop(file_opf);
            drop(file_lid);

            bytes_opf = 0;
            bytes_lid = 0;
            file_idx = file_idx.wrapping_add(1);

            file_opf = open_file!("OPF");
            file_lid = open_file!("LID");

            led_blue.set_low();
            last_rotate = Instant::now();
            last_flush = Instant::now();
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
        if b < b'0' || b > b'9' {
            return None;
        }
        idx = idx * 10 + (b - b'0') as u32;
    }
    Some(idx)
}
