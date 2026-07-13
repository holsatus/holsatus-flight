//! GNSS reader for the MicoAir MG-A01 ublox module on USART3.
//!
//! Parses UBX NAV-PVT frames at 115200 baud and publishes a `GnssData` snapshot
//! on `common::signals::RAW_GNSS_DATA`. Consumers today: `odid::current_position`
//! (Open Drone ID live position) and the takeoff-fix latch in
//! `odid::operator_position`. The full common-crate `gnss_reader` would also
//! work via the `serial` IoStream registry, but this in-crate task avoids
//! standing up that abstraction for a read-only consumer.
//!
//! The parser is the same minimal UBX byte-bashing used in `bin/gps_data.rs`
//! (sync to 0xB5 0x62, read class/ID/length, payload + 2-byte checksum). We
//! deliberately do not validate the checksum: bad bytes either fail the
//! magic-sync re-sync or land in fields that get sanity-checked downstream.

use crate::resources::SensorIrqs;
use common::signals::RAW_GNSS_DATA;
use common::types::measurements::{GnssData, GnssFix, GnssTime};
use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_time::Instant;

use crate::log as ulog;
use crate::resources::GpsResources;

pub const BAUD: u32 = 115_200;

const UBX_SYNC_1: u8 = 0xB5;
const UBX_SYNC_2: u8 = 0x62;
const NAV_CLASS: u8 = 0x01;
const NAV_PVT_ID: u8 = 0x07;
const NAV_PVT_LEN: usize = 92;
const MAX_PAYLOAD: usize = 512;

#[embassy_executor::task]
pub async fn gnss_reader_task(r: GpsResources) -> ! {
    // The GPS TX pin is unused; dropping it returns the pad to the GPIO pool.
    let _ = r.tx;

    let mut cfg = UartConfig::default();
    cfg.baudrate = BAUD;

    let mut rx = match UartRx::new(r.usart, r.rx, r.dma, SensorIrqs, cfg) {
        Ok(rx) => rx,
        Err(_) => {
            ulog::log("[gnss] USART3 init FAIL -- halting");
            loop {
                embassy_time::Timer::after_secs(60).await;
            }
        }
    };
    ulog::log("[gnss] USART3 up at 115200, parsing UBX NAV-PVT");

    let mut snd = RAW_GNSS_DATA.sender();
    let mut sync = [0u8; 1];
    let mut hdr = [0u8; 4];
    let mut pbuf = [0u8; MAX_PAYLOAD + 2];

    // GPS status logging: every LOG_PERIOD_MS, plus immediately on any fix-type
    // change, so satellite acquisition is visible in the SD log (there was no
    // such telemetry before -- a no-fix outdoor attempt looked silent).
    const LOG_PERIOD_MS: u64 = 2_000;
    let mut last_log = Instant::now();
    let mut last_fix_u8 = u8::MAX; // != any real fix -> log the first frame

    loop {
        // Sync to UBX preamble.
        loop {
            if rx.read(&mut sync).await.is_err() {
                continue;
            }
            if sync[0] != UBX_SYNC_1 {
                continue;
            }
            if rx.read(&mut sync).await.is_err() {
                continue;
            }
            if sync[0] == UBX_SYNC_2 {
                break;
            }
        }

        if rx.read(&mut hdr).await.is_err() {
            continue;
        }
        let class = hdr[0];
        let id = hdr[1];
        let len = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;
        if class != NAV_CLASS || len > MAX_PAYLOAD {
            continue;
        }
        if rx.read(&mut pbuf[..len + 2]).await.is_err() {
            continue;
        }

        if id == NAV_PVT_ID && len == NAV_PVT_LEN {
            let data = parse_nav_pvt(&pbuf);
            let (fix, sats, hacc) = (data.fix, data.num_satellites, data.horizontal_accuracy);
            let (lat_raw, lon_raw, alt) =
                (data.latitude_raw, data.longitude_raw, data.height_above_msl);
            snd.send(data);

            let now = Instant::now();
            let fix_u8 = fix as u8;
            if fix_u8 != last_fix_u8
                || now.duration_since(last_log).as_millis() as u64 >= LOG_PERIOD_MS
            {
                last_fix_u8 = fix_u8;
                last_log = now;
                // lat/lon are i32 in 1e-7 deg; cast to f64 before scaling so the
                // 7 fractional digits survive (f32 only holds ~7 sig figs total).
                // `pos=lat,lon` is the exact Google Maps paste format -- copy the
                // value after `pos=` straight into the search box.
                let mut s: heapless::String<128> = heapless::String::new();
                let _ = core::fmt::Write::write_fmt(
                    &mut s,
                    format_args!(
                        "[gnss] fix={} sats={} hacc={:.1}m pos={:.7},{:.7} alt={:.1}m",
                        fix_label(fix),
                        sats,
                        hacc,
                        lat_raw as f64 * 1e-7,
                        lon_raw as f64 * 1e-7,
                        alt,
                    ),
                );
                ulog::log(s.as_str());
            }
        }
    }
}

fn fix_label(fix: GnssFix) -> &'static str {
    match fix {
        GnssFix::NoFix => "NoFix",
        GnssFix::TimeOnly => "TimeOnly",
        GnssFix::Fix2D => "2D",
        GnssFix::Fix3D => "3D",
    }
}

fn parse_nav_pvt(p: &[u8]) -> GnssData {
    let fix = match p[20] {
        2 => GnssFix::Fix2D,
        3 | 4 => GnssFix::Fix3D,
        5 => GnssFix::TimeOnly,
        _ => GnssFix::NoFix,
    };

    GnssData {
        timestamp_us: Instant::now().as_micros(),
        time: GnssTime {
            year: u16::from_le_bytes([p[4], p[5]]),
            month: p[6],
            day: p[7],
            hour: p[8],
            min: p[9],
            sec: p[10],
        },
        fix,
        num_satellites: p[23],
        longitude_raw: le_i32(p, 24),
        latitude_raw: le_i32(p, 28),
        // NAV-PVT.hMSL is in millimetres; GnssData.height_above_msl is metres.
        height_above_msl: le_i32(p, 36) as f32 * 1e-3,
        horizontal_accuracy: le_u32(p, 40) as f32 * 1e-3,
        vertical_accuracy: le_u32(p, 44) as f32 * 1e-3,
        velocity_north: le_i32(p, 48) as f32 * 1e-3,
        velocity_east: le_i32(p, 52) as f32 * 1e-3,
        velocity_down: le_i32(p, 56) as f32 * 1e-3,
        ground_speed: le_i32(p, 60) as f32 * 1e-3,
        // NAV-PVT.headMot is degrees * 1e5; GnssData.heading_motion is radians.
        heading_motion: (le_i32(p, 64) as f32 * 1e-5).to_radians(),
        heading_accuracy: (le_u32(p, 72) as f32 * 1e-5).to_radians(),
        ground_speed_accuracy: le_u32(p, 68) as f32 * 1e-3,
        mag_declination: 0.0,
    }
}

fn le_i32(p: &[u8], off: usize) -> i32 {
    i32::from_le_bytes([p[off], p[off + 1], p[off + 2], p[off + 3]])
}

fn le_u32(p: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([p[off], p[off + 1], p[off + 2], p[off + 3]])
}
