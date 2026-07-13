//! Phoenix Pi-companion MAVLink telemetry over USART2.
//!
//! Streams the FC-side topics defined in `references/holsatus_data_logging.md`
//! to a Raspberry Pi 5 running `phoenix-logger`. The link rides on the board's
//! DJI-VTX 6-pin connector (PA2 = Tx2, PA3 = Rx2), which we repurpose from FPV
//! while this drone is in mission-logging duty.
//!
//! What this task emits (FC -> Pi), per the design doc section 8 rate budget:
//!   HEARTBEAT          @ 1 Hz   - arm/mode + companion arming-gate input
//!   SCALED_IMU         @ 100 Hz - BMI088 gyro+accel (Pi: /fc/imu/0)
//!   ATTITUDE           @ 50 Hz  - Madgwick euler+rate (Pi: /fc/attitude)
//!   LOCAL_POSITION_NED @ 50 Hz  - ESKF pos+vel (Pi: /fc/local_position)
//!   RC_CHANNELS        @ 50 Hz  - raw RC channels + RSSI (Pi: /fc/rc_input)
//!   DISTANCE_SENSOR    @ 25 Hz  - MTF-01 lidar (Pi: /fc/rangefinder)
//!   RAW_IMU            @ 20 Hz  - mag-only via xmag/ymag/zmag (Pi: /fc/mag)
//!   GPS_RAW_INT        @ 5 Hz   - ublox NAV-PVT (Pi: /fc/gps)
//!   NAMED_VALUE_INT    @ 5 Hz   - mission-FSM state "FSM" (see fsm_state.rs)
//!   BATTERY_STATUS     @ 2 Hz   - filtered pack mV (Pi: /fc/battery)
//!
//! Deliberately deferred for v1 (no in-firmware signal exists today):
//!   SCALED_IMU2        BMI270 -- bmi270_logger_task writes SD-only, no Watch
//!   SCALED_PRESSURE    DPS310 -- alt_hold reads baro but doesn't publish it
//!   OPTICAL_FLOW_RAD   MTF-01 flow is already velocity-integrated in
//!                      FLOW_VEL_MS; the raw integrated_x/y_rad fields the Pi
//!                      expects aren't preserved
//!
//! What this task receives (Pi -> FC):
//!   HEARTBEAT from compid 191 -> latches `COMPANION_HEALTHY` / `LAST_HB_US`
//!                                so a future arming gate can read it
//!   TIMESYNC tc1==0           -> echo back with our local time as tc1; the
//!                                Pi uses the round-trip to estimate FC<->Pi
//!                                clock offset post-mission
//!
//! UART config: 921600 8N1 on USART2 PA2/PA3 with DMA2_CH5 (TX) / DMA2_CH6 (RX).
//! Buffered DMA on both directions; the TX path never blocks the 10 ms tick
//! beyond the time it takes to push ~50 bytes at 921 kbps (~0.5 ms worst case).

use core::num::Wrapping;
use core::sync::atomic::{AtomicBool, Ordering};

// AtomicI64 / AtomicU64 are not in core on Cortex-M; portable-atomic falls
// back to a critical-section impl that is sound on this target.
use portable_atomic::{AtomicI64, AtomicU64};

use common::signals;
use common::types::measurements::GnssFix;
use embassy_futures::join::join;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx, UartTx};
use embassy_time::{Duration, Instant, Ticker, Timer};
use mavio::dialects::common::enums::MavSeverity;
use mavio::dialects::common::messages::{
    Attitude, BatteryStatus, DistanceSensor, GpsRawInt, Heartbeat, LocalPositionNed, NamedValueInt,
    RawImu, RcChannels, ScaledImu, ScaledImu2, Statustext, Timesync,
};
use mavio::dialects::minimal::enums::{MavAutopilot, MavModeFlag, MavState, MavType};
use mavio::prelude::{V2, Versioned};
use mavio::protocol::FrameParser;

use crate::alt_hold::LIDAR_ALT_M;
use crate::battery::BATTERY_FILTERED_MV;
use crate::log as ulog;
use crate::resources::{TelemIrqs, TelemResources};

pub const BAUD: u32 = 921_600;

/// MAVLink identity for the FC side of the link.
const SYS_ID: u8 = 1;
/// MAV_COMP_ID_AUTOPILOT1
const COMP_ID_AUTOPILOT: u8 = 1;
/// MAV_COMP_ID_ONBOARD_COMPUTER -- what the Pi-side logger advertises.
const COMP_ID_COMPANION: u8 = 191;

/// True iff a HEARTBEAT from `COMP_ID_COMPANION` with `MAV_STATE_ACTIVE` has
/// arrived within `COMPANION_TIMEOUT_MS`. A future arming gate can read this
/// (default-open today: nothing in the flight pipeline blocks on it yet).
pub static COMPANION_HEALTHY: AtomicBool = AtomicBool::new(false);

/// Embassy `Instant::as_micros()` of the most recent companion HEARTBEAT.
/// 0 means none received this boot.
pub static LAST_COMPANION_HB_US: AtomicU64 = AtomicU64::new(0);

/// Most recent TIMESYNC round-trip offset estimate (ns, FC ahead of Pi).
/// Updated whenever the Pi sends a TIMESYNC response (tc1 != 0 && ts1
/// matches one of ours -- we don't currently initiate, so this stays 0
/// unless the Pi-side `_send_timesync` reaches us. Reserved for future use).
pub static TIMESYNC_OFFSET_NS: AtomicI64 = AtomicI64::new(0);

/// Companion is considered unhealthy if no qualifying HEARTBEAT was observed
/// within this many milliseconds.
const COMPANION_TIMEOUT_MS: u64 = 2_000;

/// Standard gravity used for accel m/s^2 -> milli-g conversion in SCALED_IMU.
const G: f32 = 9.806_65;

#[embassy_executor::task]
pub async fn phoenix_telem_task(r: TelemResources) -> ! {
    // First line uses log_critical (priority channel, drained first by
    // uart_writer_task) so it survives even when [cal] floods the regular
    // ulog channel at boot. If we see this in the SD log, the task got
    // scheduled and reached its first await. If we don't, the scheduler
    // never woke it, or it panicked before reaching here.
    ulog::log_critical("[phoenix] task entered").await;

    let mut cfg = UartConfig::default();
    cfg.baudrate = BAUD;

    // log_reliable blocks until queued, so missing this means we panicked
    // between "task entered" and here -- almost certainly during the
    // UartConfig setup, which would be weird (it's stack-only).
    ulog::log_reliable("[phoenix] initializing USART2 (PA2 TX / PA3 RX)").await;

    // embassy_stm32::usart::Uart::new signature is (peri, rx, tx, tx_dma,
    // rx_dma, irq, config) -- note irq is 6th, AFTER both DMA channels.
    let uart = match Uart::new(r.usart, r.rx, r.tx, r.dma_tx, r.dma_rx, TelemIrqs, cfg) {
        Ok(u) => u,
        Err(_) => {
            // Critical priority so this surfaces immediately even under
            // log saturation. The task halts here in a sleep loop.
            ulog::log_critical("[phoenix] USART2 init FAIL -- task halted").await;
            loop {
                Timer::after_secs(60).await;
            }
        }
    };
    ulog::log_critical("[phoenix] USART2 up @ 921600").await;

    let (tx, rx) = uart.split();
    ulog::log_reliable("[phoenix] UART split; entering tx_loop / rx_loop").await;

    // Run TX scheduler and RX parser concurrently. Either future is meant to
    // be `-> !`, so this join never returns, but the type system can't see
    // that across split halves -- the `loop`/`unreachable!()` is for the
    // compiler.
    join(tx_loop(tx), rx_loop(rx)).await;
    unreachable!()
}

// ---------------------------------------------------------------------------
// TX path -- periodic stream of FC -> Pi telemetry.
// ---------------------------------------------------------------------------

async fn tx_loop(mut tx: UartTx<'static, Async>) -> ! {
    let mut seq: Wrapping<u8> = Wrapping(0);
    let mut tick: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        ticker.next().await;
        tick = tick.wrapping_add(1);

        // 100 Hz -- both IMUs.
        if let Some(msg) = build_scaled_imu() {
            send(&mut tx, &mut seq, &msg).await;
        }
        if let Some(msg) = build_scaled_imu2() {
            send(&mut tx, &mut seq, &msg).await;
        }

        // 50 Hz
        if tick % 2 == 0 {
            if let Some(msg) = build_attitude() {
                send(&mut tx, &mut seq, &msg).await;
            }
            if let Some(msg) = build_local_position_ned() {
                send(&mut tx, &mut seq, &msg).await;
            }
            if let Some(msg) = build_rc_channels() {
                send(&mut tx, &mut seq, &msg).await;
            }
        }

        // 25 Hz
        if tick % 4 == 0 {
            if let Some(msg) = build_distance_sensor() {
                send(&mut tx, &mut seq, &msg).await;
            }
        }

        // 20 Hz
        if tick % 5 == 0 {
            if let Some(msg) = build_raw_imu_mag() {
                send(&mut tx, &mut seq, &msg).await;
            }
        }

        // 5 Hz -- mission-FSM state, for following mode/arm transitions live.
        if tick % 20 == 0 {
            send(&mut tx, &mut seq, &build_named_value_int_fsm()).await;
        }

        // 5 Hz
        if tick % 20 == 0 {
            if let Some(msg) = build_gps_raw_int() {
                send(&mut tx, &mut seq, &msg).await;
            }
        }

        // 2 Hz
        if tick % 50 == 0 {
            if let Some(msg) = build_battery_status() {
                send(&mut tx, &mut seq, &msg).await;
            }
        }

        // Drain any pending TIMESYNC response from the RX path. Done every
        // tick so the round-trip stays tight (<= 10 ms FC turnaround) and
        // the Pi's offset estimate isn't biased by our scheduling jitter.
        let tc1 = TIMESYNC_PENDING.swap(0, Ordering::Relaxed);
        if tc1 != 0 {
            let ts1 = TIMESYNC_PENDING_TS1.load(Ordering::Relaxed);
            let mut m = Timesync::default();
            m.tc1 = tc1;
            m.ts1 = ts1;
            send(&mut tx, &mut seq, &m).await;
        }

        // Mirror bridged ulog lines (FSM, sensor init, [phoenix] link
        // events, ...) as MAVLink STATUSTEXT so the Pi-side mavlog pane
        // surfaces them next to the heartbeats. Bounded to 8 per tick so
        // a sudden burst can't starve the periodic telemetry topics
        // above; at ~66 bytes per STATUSTEXT wire-frame and 8/tick that's
        // ~52 KB/s peak -- well under the 92 KB/s link budget. See also
        // `ulog::TELEM_CHANNEL` for the producer side.
        for _ in 0..8 {
            match ulog::TELEM_CHANNEL.try_receive() {
                Ok(line) => send(&mut tx, &mut seq, &build_statustext(line.as_str())).await,
                Err(_) => break,
            }
        }

        // 2 Hz heartbeat + companion-health timeout sweep. Doc-doc says 1 Hz
        // is enough; ground-station tools like mavproxy flag "link down" if a
        // single heartbeat slips past the 2.5-3 s timeout, so doubling the
        // rate gives us a comfortable margin against Ticker jitter at no
        // bandwidth cost (~30 B/s).
        if tick % 50 == 0 {
            send(&mut tx, &mut seq, &build_heartbeat()).await;
            sweep_companion_timeout();
        }
    }
}

/// Serialize a mavio message as a V2 frame from (SYS_ID, COMP_ID_AUTOPILOT)
/// and shove it out the UART. Errors are dropped intentionally -- a single
/// bad payload should not stall the stream.
async fn send<M: mavio::Message>(
    tx: &mut UartTx<'static, Async>,
    seq: &mut Wrapping<u8>,
    msg: &M,
) {
    let frame = mavio::Frame::builder()
        .system_id(SYS_ID)
        .component_id(COMP_ID_AUTOPILOT)
        .sequence(seq.0)
        .version(V2::v())
        .message(msg);
    let frame = match frame {
        Ok(b) => b.build(),
        Err(_) => return,
    };
    let mut buf = [0u8; 280];
    let n = match frame.serialize(&mut buf) {
        Ok(n) => n,
        Err(_) => return,
    };
    let _ = tx.write(&buf[..n]).await;
    *seq += 1;
}

// ---------------------------------------------------------------------------
// Message builders -- one per topic. Each pulls from the live Watch / atomic
// signal published elsewhere in the firmware. Returning `None` skips the
// emit for this tick, which the Pi sees as a transient gap (V&V gap check
// allows up to 10% drop per topic).
// ---------------------------------------------------------------------------

fn build_heartbeat() -> Heartbeat {
    let armed = signals::MOTORS_STATE
        .try_get()
        .map(|s| s.is_armed())
        .unwrap_or(false);

    let mut base_mode = MavModeFlag::CUSTOM_MODE_ENABLED;
    if armed {
        base_mode |= MavModeFlag::SAFETY_ARMED;
    }

    Heartbeat {
        type_: MavType::Quadrotor,
        autopilot: MavAutopilot::Generic,
        base_mode,
        // Mission-FSM state code (see fsm_state.rs for the legend). The web
        // stream shows this in every HEARTBEAT alongside the dedicated
        // NAMED_VALUE_INT "FSM" line.
        custom_mode: crate::fsm_state::current_code() as u32,
        system_status: if armed {
            MavState::Active
        } else {
            MavState::Standby
        },
        mavlink_version: 3,
    }
}

fn build_scaled_imu() -> Option<ScaledImu> {
    // BMI088 lives at index 0 in the multi-IMU array. Note: the singular
    // CAL_IMU_DATA Watch exists in common::signals but nothing publishes to
    // it on H743v2 -- the live pipeline uses CAL_MULTI_IMU_DATA[idx] (see
    // common::tasks::imu_reader::main_6dof).
    let imu = signals::CAL_MULTI_IMU_DATA[0].try_get()?;
    let mut m = ScaledImu::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    // Pi parses xacc/yacc/zacc as: m.xacc / 1000.0 * 9.80665 -> m/s^2.
    // So encode m/s^2 -> milli-g (int16).
    m.xacc = clamp_i16(imu.acc[0] / G * 1000.0);
    m.yacc = clamp_i16(imu.acc[1] / G * 1000.0);
    m.zacc = clamp_i16(imu.acc[2] / G * 1000.0);
    // Pi parses xgyro/ygyro/zgyro as: m.xgyro / 1000.0 -> rad/s. So encode
    // rad/s -> milli-rad/s (int16). MAVLink spec agrees.
    m.xgyro = clamp_i16(imu.gyr[0] * 1000.0);
    m.ygyro = clamp_i16(imu.gyr[1] * 1000.0);
    m.zgyro = clamp_i16(imu.gyr[2] * 1000.0);
    Some(m)
}

fn build_scaled_imu2() -> Option<ScaledImu2> {
    // BMI270 at index 1. Same scaling convention as IMU0.
    let imu = signals::CAL_MULTI_IMU_DATA[1].try_get()?;
    let mut m = ScaledImu2::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    m.xacc = clamp_i16(imu.acc[0] / G * 1000.0);
    m.yacc = clamp_i16(imu.acc[1] / G * 1000.0);
    m.zacc = clamp_i16(imu.acc[2] / G * 1000.0);
    m.xgyro = clamp_i16(imu.gyr[0] * 1000.0);
    m.ygyro = clamp_i16(imu.gyr[1] * 1000.0);
    m.zgyro = clamp_i16(imu.gyr[2] * 1000.0);
    Some(m)
}

fn build_raw_imu_mag() -> Option<RawImu> {
    // CAL_MULTI_MAG_DATA is [Watch<[f32; 3]>; NUM_MAG] -- just the mag
    // triplet (gauss, post hard/soft-iron cal), NOT an Imu9DofData. The Pi's
    // /fc/mag handler only consumes xmag/ymag/zmag.
    let mag = signals::CAL_MULTI_MAG_DATA[0].try_get()?;
    let mut m = RawImu::default();
    m.time_usec = Instant::now().as_micros();
    m.xmag = clamp_i16(mag[0] * 1000.0);
    m.ymag = clamp_i16(mag[1] * 1000.0);
    m.zmag = clamp_i16(mag[2] * 1000.0);
    Some(m)
}

fn build_attitude() -> Option<Attitude> {
    let att = signals::AHRS_ATTITUDE.try_get()?;
    let gyr = signals::CAL_MULTI_IMU_DATA[0]
        .try_get()
        .map(|i| i.gyr)
        .unwrap_or([0.0; 3]);
    let mut m = Attitude::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    [m.roll, m.pitch, m.yaw] = att;
    [m.rollspeed, m.pitchspeed, m.yawspeed] = gyr;
    Some(m)
}

fn build_local_position_ned() -> Option<LocalPositionNed> {
    let est = signals::ESKF_ESTIMATE.try_get()?;
    let mut m = LocalPositionNed::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    m.x = est.pos[0];
    m.y = est.pos[1];
    m.z = est.pos[2];
    m.vx = est.vel[0];
    m.vy = est.vel[1];
    m.vz = est.vel[2];
    Some(m)
}

fn build_rc_channels() -> Option<RcChannels> {
    let chans = signals::RC_CHANNELS_RAW.try_get()??;
    // RcStatus carries `quality` (0-100, ELRS link quality). MAVLink RC_CHANNELS
    // wants `rssi` as 0-254 (255 = unknown). Scale linearly; saturate to 254.
    let rssi = signals::RC_STATUS
        .try_get()
        .map(|s| ((s.quality as u16 * 254) / 100).min(254) as u8)
        .unwrap_or(255);
    let mut m = RcChannels::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    m.chancount = 16;
    m.chan1_raw = chans[0];
    m.chan2_raw = chans[1];
    m.chan3_raw = chans[2];
    m.chan4_raw = chans[3];
    m.chan5_raw = chans[4];
    m.chan6_raw = chans[5];
    m.chan7_raw = chans[6];
    m.chan8_raw = chans[7];
    m.chan9_raw = chans[8];
    m.chan10_raw = chans[9];
    m.chan11_raw = chans[10];
    m.chan12_raw = chans[11];
    m.chan13_raw = chans[12];
    m.chan14_raw = chans[13];
    m.chan15_raw = chans[14];
    m.chan16_raw = chans[15];
    m.rssi = rssi;
    Some(m)
}

fn build_distance_sensor() -> Option<DistanceSensor> {
    use mavio::dialects::common::enums::{MavDistanceSensor, MavSensorOrientation};
    let alt_m = LIDAR_ALT_M.try_get()?;
    let mut m = DistanceSensor::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    // MTF-01 spec: 0-8 m AGL.
    m.min_distance = 0;
    m.max_distance = 800;
    m.current_distance = if alt_m < 0.0 { 0 } else { (alt_m * 100.0) as u16 };
    m.type_ = MavDistanceSensor::Laser;
    m.id = 0;
    // Downward-facing rangefinder. The mavio dialect generator preserves the
    // raw enum-name prefix here, hence the long variant name.
    m.orientation = MavSensorOrientation::MavSensorRotationPitch270;
    m.covariance = 255; // unknown
    Some(m)
}

fn build_gps_raw_int() -> Option<GpsRawInt> {
    let g = signals::RAW_GNSS_DATA.try_get()?;
    let mut m = GpsRawInt::default();
    m.time_usec = Instant::now().as_micros();
    m.fix_type = match g.fix {
        GnssFix::NoFix => mavio::dialects::common::enums::GpsFixType::NoFix,
        GnssFix::TimeOnly => mavio::dialects::common::enums::GpsFixType::NoFix,
        GnssFix::Fix2D => mavio::dialects::common::enums::GpsFixType::_2dFix,
        GnssFix::Fix3D => mavio::dialects::common::enums::GpsFixType::_3dFix,
    };
    m.lat = g.latitude_raw;
    m.lon = g.longitude_raw;
    m.alt = (g.height_above_msl * 1000.0) as i32; // mm
    m.eph = clamp_u16(g.horizontal_accuracy * 100.0); // cm
    m.epv = clamp_u16(g.vertical_accuracy * 100.0);
    m.vel = clamp_u16(g.ground_speed * 100.0);
    m.cog = clamp_u16(g.heading_motion.to_degrees() * 100.0); // centidegrees
    m.satellites_visible = g.num_satellites;
    Some(m)
}

fn build_named_value_int_fsm() -> NamedValueInt {
    // Mission-FSM state as a labeled scalar topic. The web stream auto-lists
    // each NAMED_VALUE_* by name, so this appears as its own "FSM" line. Value
    // is the wire code defined in fsm_state.rs (0=GroundIdle .. 5=Fault).
    let mut m = NamedValueInt::default();
    m.time_boot_ms = (Instant::now().as_millis() & 0xFFFF_FFFF) as u32;
    m.name[..3].copy_from_slice(b"FSM");
    m.value = crate::fsm_state::current_code() as i32;
    m
}

fn build_statustext(msg: &str) -> Statustext {
    // STATUSTEXT.text is char[50], not null-terminated. Truncate at a UTF-8
    // boundary via the shared helper so a 96-byte ulog line doesn't split
    // a multi-byte char on the wire. Severity is Info for v1; we can grow
    // a per-prefix mapping (ERR / WARN / FAIL -> Error / Warning) once
    // the bench operator actually wants that.
    let trimmed = common::utils::string_trunc::truncate_to_byte_cap(msg, 50);
    let bytes = trimmed.as_bytes();
    let mut m = Statustext::default();
    m.severity = MavSeverity::Info;
    m.text[..bytes.len()].copy_from_slice(bytes);
    // id = 0 + chunk_seq = 0 indicates a single-chunk message; we never
    // need to reassemble across STATUSTEXTs because we already truncate.
    m
}

fn build_battery_status() -> Option<BatteryStatus> {
    let mv = BATTERY_FILTERED_MV.try_get()?;
    let mut m = BatteryStatus::default();
    // The Pi sums all 0 < v < 65535 cell entries for `voltage_mv`. We don't
    // have per-cell sense on this board, so report the whole 4S pack in
    // cell 0 and leave the rest at the unset sentinel (65535 = "no cell").
    m.voltages[0] = mv as u16;
    for i in 1..10 {
        m.voltages[i] = u16::MAX;
    }
    m.current_battery = -1; // current sensor not wired through to firmware
    m.current_consumed = -1;
    m.energy_consumed = -1;
    m.battery_remaining = -1;
    m.temperature = i16::MAX;
    Some(m)
}

// ---------------------------------------------------------------------------
// RX path -- parse MAVLink from Pi, handle the two messages we care about.
// ---------------------------------------------------------------------------

async fn rx_loop(mut rx: UartRx<'static, Async>) -> ! {
    let mut parser: FrameParser<V2> = FrameParser::new();

    loop {
        let buf = parser.buffer_to_fill();
        // read_until_idle: fills up to `buf.len()` and returns early on
        // UART-idle. Lets the parser see bytes as they arrive without
        // waiting for the parser's exact requested chunk to land.
        let n = match rx.read_until_idle(buf).await {
            Ok(n) if n > 0 => n,
            Ok(_) => continue,
            Err(_) => {
                // Read error (framing/noise/idle while DMA was off): reset
                // parser state and back off so we don't spin on a wedged
                // UART. The Pi will retry its messages.
                parser = FrameParser::new();
                Timer::after_millis(50).await;
                continue;
            }
        };
        if let Some(frame) = parser.commit_bytes(n) {
            handle_frame(frame);
        }
    }
}

fn handle_frame(frame: mavio::Frame<V2>) {
    use mavio::dialects::common::messages as m;

    match frame.message_id() {
        Heartbeat::ID => {
            if frame.component_id() != COMP_ID_COMPANION {
                return;
            }
            let Ok(msg) = frame.decode_message::<m::Heartbeat>() else {
                return;
            };
            // Active = STANDBY or RECORDING per Pi-side state machine.
            // CRITICAL would mean ERROR; both still indicate the logger is
            // alive, but only Active marks "ready" per design doc 11.1.
            if msg.system_status == MavState::Active {
                LAST_COMPANION_HB_US.store(Instant::now().as_micros(), Ordering::Relaxed);
                let was_healthy = COMPANION_HEALTHY.swap(true, Ordering::Relaxed);
                if !was_healthy {
                    ulog::log("[phoenix] companion link UP");
                }
            }
        }
        Timesync::ID => {
            let Ok(msg) = frame.decode_message::<m::Timesync>() else {
                return;
            };
            // tc1 != 0 means this is a *response* to a sync we initiated. We
            // don't initiate today, so just stash the offset estimate and
            // move on.
            if msg.tc1 != 0 {
                let now_ns = (Instant::now().as_micros() as i64).saturating_mul(1000);
                let offset = (msg.tc1 - msg.ts1) - (now_ns - msg.ts1) / 2;
                TIMESYNC_OFFSET_NS.store(offset, Ordering::Relaxed);
                return;
            }
            // tc1 == 0: ping request, the Pi wants us to echo with our own
            // local time so it can estimate our clock. We need TX access to
            // respond. Defer to the TX path via TIMESYNC_PENDING (a tiny
            // shared slot) so we don't have to share `tx` between the two
            // halves of the split UART.
            let now_ns = (Instant::now().as_micros() as i64).saturating_mul(1000);
            TIMESYNC_PENDING.store(now_ns, Ordering::Relaxed);
            TIMESYNC_PENDING_TS1.store(msg.ts1, Ordering::Relaxed);
        }
        _ => { /* ignore */ }
    }
}

/// One-deep mailbox from RX -> TX for TIMESYNC responses. Non-zero in `tc1`
/// slot means "TX path: please send a TIMESYNC with these fields, then zero
/// this back out." If a second request comes in before TX drains the first,
/// the older one is overwritten -- losing a sync is benign because the Pi
/// retransmits at ~0.5 Hz by default.
static TIMESYNC_PENDING: AtomicI64 = AtomicI64::new(0);
static TIMESYNC_PENDING_TS1: AtomicI64 = AtomicI64::new(0);

fn sweep_companion_timeout() {
    let last_us = LAST_COMPANION_HB_US.load(Ordering::Relaxed);
    if last_us == 0 {
        return;
    }
    let age_ms = Instant::now().as_micros().saturating_sub(last_us) / 1000;
    if age_ms > COMPANION_TIMEOUT_MS {
        let was_healthy = COMPANION_HEALTHY.swap(false, Ordering::Relaxed);
        if was_healthy {
            ulog::log("[phoenix] companion link DOWN (heartbeat timeout)");
        }
    }
}

// ---------------------------------------------------------------------------
// Small numeric helpers used across builders.
// ---------------------------------------------------------------------------

fn clamp_i16(v: f32) -> i16 {
    if v.is_nan() {
        0
    } else if v > i16::MAX as f32 {
        i16::MAX
    } else if v < i16::MIN as f32 {
        i16::MIN
    } else {
        v as i16
    }
}

fn clamp_u16(v: f32) -> u16 {
    if v.is_nan() || v < 0.0 {
        0
    } else if v > u16::MAX as f32 {
        u16::MAX
    } else {
        v as u16
    }
}
