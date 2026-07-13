//! Open Drone ID transmitter for ArduRemoteID Supermini.
//!
//! Owns UART5 (PB6 TX) at 57600 baud and streams a HEARTBEAT plus the five
//! OPEN_DRONE_ID_* MAVLink messages at 1 Hz so the ArduRemoteID module has
//! everything it needs to broadcast a DIN EN 4709-002 / ASD-STAN compliant
//! BT4 Legacy frame on every cycle.
//!
//! Operator and aircraft identity are baked in via the constants at the top
//! of this module. The values mirror `tools/odid_emit/secrets.toml` and must
//! be set before flashing for the broadcast to be lawful in the EU.
//!
//! Location data is hard-coded today. Once `RAW_GNSS_DATA` lock is reliable
//! in flight, swap `current_position` and `operator_position` for live reads.

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Instant, Timer};
use mavio::dialects::common::enums::{
    MavOdidCategoryEu, MavOdidClassEu, MavOdidClassificationType, MavOdidDescType,
    MavOdidHeightRef, MavOdidHorAcc, MavOdidIdType, MavOdidOperatorIdType,
    MavOdidOperatorLocationType, MavOdidSpeedAcc, MavOdidStatus, MavOdidTimeAcc,
    MavOdidUaType, MavOdidVerAcc,
};
use mavio::dialects::common::messages::{
    Heartbeat, OpenDroneIdBasicId, OpenDroneIdLocation, OpenDroneIdOperatorId,
    OpenDroneIdSelfId, OpenDroneIdSystem,
};
use mavio::dialects::minimal::enums::{MavAutopilot, MavState, MavType};
use mavio::prelude::{V2, Versioned};

use crate::log as ulog;
use crate::resources::{OdidIrqs, OdidResources};

// Operator identity. Values are read at build time from the gitignored
// `tools/odid_emit/secrets.toml` (see build.rs `emit_odid_secrets`). If the
// file is missing or a key is absent, the placeholder below is used and the
// broadcast is non-compliant. The same TOML feeds the Python smoke test, so
// there is exactly one source of truth for identity.
pub const OPERATOR_ID: &str = match option_env!("ODID_OPERATOR_ID") {
    Some(s) => s,
    None    => "DEU_REPLACE_ME",
};
pub const UAS_ID: &str = match option_env!("ODID_UAS_ID") {
    Some(s) => s,
    None    => "HOLSATUS-H743V2-0001",
};
pub const SELF_ID: &str = match option_env!("ODID_SELF_ID") {
    Some(s) => s,
    None    => "Holsatus Phoenix",
};

// Default takeoff / vehicle position used until GNSS lock is wired in.
// Treptower Park, Berlin: the regular flying site.
pub const FALLBACK_LAT_DEG: f64 = 52.486_473_0;
pub const FALLBACK_LON_DEG: f64 = 13.471_140_7;
pub const FALLBACK_ALT_M:   f32 = 0.0;

pub const BAUD: u32 = 57_600;
pub const SYSTEM_ID: u8 = 1;
pub const COMPONENT_ID: u8 = 1;

// Unix seconds at 2019-01-01 00:00:00 UTC. OPEN_DRONE_ID_SYSTEM.timestamp is
// measured from this epoch.
const ODID_EPOCH_UNIX: u32 = 1_546_300_800;

#[embassy_executor::task]
pub async fn odid_tx_task(r: OdidResources) -> ! {
    let mut cfg = UartConfig::default();
    cfg.baudrate = BAUD;

    let mut uart = match UartTx::new(r.usart, r.tx, r.dma, OdidIrqs, cfg) {
        Ok(u) => u,
        Err(_) => {
            ulog::log("[odid] UART5 init FAIL -- halting");
            loop { Timer::after_secs(60).await; }
        }
    };

    ulog::log("[odid] UART5 up at 57600, broadcasting at 1Hz");

    let mut seq: u8 = 0;
    let mut buf = [0u8; 280];

    loop {
        let now_us = Instant::now().as_micros();
        let unix_now = ODID_EPOCH_UNIX.wrapping_add((now_us / 1_000_000) as u32);
        let (lat_raw, lon_raw, alt_geo_m) = current_position();
        let (op_lat_raw, op_lon_raw) = operator_position();

        send(&mut uart, &mut seq, &mut buf, &heartbeat()).await;
        send(&mut uart, &mut seq, &mut buf, &basic_id()).await;
        send(&mut uart, &mut seq, &mut buf, &operator_id()).await;
        send(&mut uart, &mut seq, &mut buf, &self_id()).await;
        send(&mut uart, &mut seq, &mut buf,
             &system(op_lat_raw, op_lon_raw, unix_now)).await;
        send(&mut uart, &mut seq, &mut buf,
             &location(lat_raw, lon_raw, alt_geo_m, now_us)).await;

        Timer::after_millis(1000).await;
    }
}

async fn send(
    uart: &mut UartTx<'static, embassy_stm32::mode::Async>,
    seq: &mut u8,
    buf: &mut [u8],
    message: &dyn mavio::Message,
) {
    let Ok(builder) = mavio::Frame::builder()
        .system_id(SYSTEM_ID)
        .component_id(COMPONENT_ID)
        .sequence(*seq)
        .version(V2::v())
        .message(message)
    else {
        return;
    };
    let frame = builder.build();

    let n = match frame.serialize(buf) {
        Ok(n) => n,
        Err(_) => return,
    };

    let _ = uart.write(&buf[..n]).await;
    *seq = seq.wrapping_add(1);
}

fn pad<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = s.as_bytes();
    let n = bytes.len().min(N);
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

fn heartbeat() -> Heartbeat {
    Heartbeat {
        type_: MavType::Quadrotor,
        autopilot: MavAutopilot::Generic,
        base_mode: Default::default(),
        custom_mode: 0,
        system_status: MavState::Active,
        mavlink_version: 3,
    }
}

fn basic_id() -> OpenDroneIdBasicId {
    OpenDroneIdBasicId {
        target_system: 0,
        target_component: 0,
        id_or_mac: [0; 20],
        id_type: MavOdidIdType::SerialNumber,
        ua_type: MavOdidUaType::HelicopterOrMultirotor,
        uas_id: pad(UAS_ID),
    }
}

fn operator_id() -> OpenDroneIdOperatorId {
    OpenDroneIdOperatorId {
        target_system: 0,
        target_component: 0,
        id_or_mac: [0; 20],
        operator_id_type: MavOdidOperatorIdType::Caa,
        operator_id: pad(OPERATOR_ID),
    }
}

fn self_id() -> OpenDroneIdSelfId {
    OpenDroneIdSelfId {
        target_system: 0,
        target_component: 0,
        id_or_mac: [0; 20],
        description_type: MavOdidDescType::Text,
        description: pad(SELF_ID),
    }
}

fn system(op_lat_raw: i32, op_lon_raw: i32, unix_now: u32) -> OpenDroneIdSystem {
    OpenDroneIdSystem {
        target_system: 0,
        target_component: 0,
        id_or_mac: [0; 20],
        operator_location_type: MavOdidOperatorLocationType::Takeoff,
        classification_type: MavOdidClassificationType::Eu,
        operator_latitude: op_lat_raw,
        operator_longitude: op_lon_raw,
        area_count: 1,
        area_radius: 0,
        area_ceiling: -1000.0,
        area_floor: -1000.0,
        category_eu: MavOdidCategoryEu::Open,
        class_eu: MavOdidClassEu::Undeclared,
        operator_altitude_geo: -1000.0,
        timestamp: unix_now,
    }
}

fn location(lat_raw: i32, lon_raw: i32, alt_geo_m: f32, now_us: u64) -> OpenDroneIdLocation {
    // ODID timestamp: seconds after the full hour. With no UTC time source we
    // approximate by reducing the boot clock modulo one hour; receivers tolerate
    // skew, the field is mainly for staleness detection within a session.
    let ts = ((now_us / 1_000) % (60 * 60 * 1000)) as f32 / 1000.0;

    OpenDroneIdLocation {
        target_system: 0,
        target_component: 0,
        id_or_mac: [0; 20],
        status: MavOdidStatus::Ground,
        direction: 36100,
        speed_horizontal: 25500,
        speed_vertical: 6300,
        latitude: lat_raw,
        longitude: lon_raw,
        altitude_barometric: -1000.0,
        altitude_geodetic: alt_geo_m,
        height_reference: MavOdidHeightRef::OverTakeoff,
        height: 0.0,
        horizontal_accuracy: MavOdidHorAcc::Unknown,
        vertical_accuracy: MavOdidVerAcc::Unknown,
        barometer_accuracy: MavOdidVerAcc::Unknown,
        speed_accuracy: MavOdidSpeedAcc::Unknown,
        timestamp: ts,
        timestamp_accuracy: MavOdidTimeAcc::Unknown,
    }
}

// Takeoff position latched on the first 3D GNSS fix and reported as
// SYSTEM.operator_{latitude,longitude} for the rest of the session. A new
// flight (power-cycle) latches a fresh value.
static TAKEOFF_LAT_RAW: AtomicI32 = AtomicI32::new(0);
static TAKEOFF_LON_RAW: AtomicI32 = AtomicI32::new(0);
static TAKEOFF_LATCHED: AtomicBool = AtomicBool::new(false);

fn current_position() -> (i32, i32, f32) {
    if let Some(gnss) = common::signals::RAW_GNSS_DATA.try_get() {
        if matches!(gnss.fix, common::types::measurements::GnssFix::Fix3D) {
            if !TAKEOFF_LATCHED.load(Ordering::Acquire) {
                TAKEOFF_LAT_RAW.store(gnss.latitude_raw, Ordering::Relaxed);
                TAKEOFF_LON_RAW.store(gnss.longitude_raw, Ordering::Relaxed);
                TAKEOFF_LATCHED.store(true, Ordering::Release);
            }
            return (gnss.latitude_raw, gnss.longitude_raw, gnss.height_above_msl);
        }
    }
    (deg_to_raw(FALLBACK_LAT_DEG), deg_to_raw(FALLBACK_LON_DEG), FALLBACK_ALT_M)
}

fn operator_position() -> (i32, i32) {
    if TAKEOFF_LATCHED.load(Ordering::Acquire) {
        return (
            TAKEOFF_LAT_RAW.load(Ordering::Relaxed),
            TAKEOFF_LON_RAW.load(Ordering::Relaxed),
        );
    }
    (deg_to_raw(FALLBACK_LAT_DEG), deg_to_raw(FALLBACK_LON_DEG))
}

fn deg_to_raw(deg: f64) -> i32 {
    (deg * 1.0e7) as i32
}
