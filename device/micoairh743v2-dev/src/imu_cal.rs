//! Shared IMU accel/gyro calibration helper.
//!
//! Captures a zero-motion bias estimate and writes it into
//! `imu_reader::params::TABLE`. Used by every flight-capable binary
//! (flight, sub_hover_test, pid_sweep_test).
//!
//! Gate sequence before the capture window:
//!   1. Wait until the drone is both STATIONARY (|gyro| small) and
//!      LEVEL (|acc_x|, |acc_y| small) for `READY_HOLD_MS` continuous.
//!      This prevents capturing bias while the user is still placing
//!      the drone (D000077: plugged LiPo while still handling -> bad
//!      bias baked in -> AHRS permanently tilted ~12 deg -> mission
//!      never armed).
//!   2. Wait `CLEAR_HANDS_MS` so the user has time to pull their hands
//!      away after the ready beep.
//!   3. Capture `N` samples (~2 s at 1 kHz) and write the mean as bias.
//!
//! All thresholds are generous so a drone placed on a table easily
//! passes, but a hand-held one does not.

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use common::signals::RAW_MULTI_IMU_DATA;
use common::tasks::imu_reader;
use embassy_time::{Duration, Instant, Timer};

/// Set once apply() has captured a bias from genuinely stationary+level data
/// and returned. HIL's hil_link_task exposes this in its AttitudeFrame reply
/// (see src/bin/hil.rs) so a host script can wait for real calibration
/// instead of guessing a fixed warm-up duration -- the capture window's
/// real-time length depends on how fast the link delivers samples, which is
/// nothing like the "~2s at 1kHz" this module assumes for real hardware.
pub static CAL_DONE: AtomicBool = AtomicBool::new(false);

use crate::log as ulog;

/// Max |gyro| (rad/s) on the low-pass-filtered signal for "stationary".
/// ~2.9 deg/s. A resting drone has zero-rate bias <~1 deg/s; a hand
/// touching the drone shows 5+ deg/s of tremor. The check is against the
/// filtered signal, so raw 1 kHz noise spikes don't repeatedly reset the
/// arming timer (D000100: gate got stuck because single-sample noise
/// kept tripping a tighter 0.02 rad/s threshold).
const STILL_GYR_RADS: f32 = 0.05;

/// Max |acc_x|, |acc_y| (m/s^2) on the low-pass-filtered signal for
/// "level". ~6 deg tilt. BMI088 raw accel noise is ~0.2-0.3 m/s^2 RMS
/// with occasional 0.5+ spikes, so a tight instantaneous threshold
/// bounces; the LPF here gives us a calm signal to gate against.
const LEVEL_ACC_MS2: f32 = 1.0;

/// Exponential-LPF coefficient applied per sample (~1 kHz). 0.02 gives
/// a ~50 ms time constant -- fast enough that "hands off" is detected
/// within ~200 ms, slow enough to reject single-sample noise.
const LPF_ALPHA: f32 = 0.02;

/// The filtered ready condition must hold continuously for this long
/// before we accept "drone is placed". Debounces the setdown flutter.
const READY_HOLD_MS: u64 = 1000;

/// Heartbeat cadence during the gate. Prints the filtered gyr/acc so
/// the operator can see why the gate hasn't armed yet, and also drives
/// enough log messages through CHANNEL to force uart_writer's flushing
/// (so the progress is actually visible on miniterm + SD).
const HEARTBEAT_MS: u64 = 500;

/// After ready, wait this long so the user's hands can clear the
/// airframe before we freeze the bias. User requested 5 s.
const CLEAR_HANDS_MS: u64 = 5000;

/// Sample count for the actual bias capture. 2000 samples at 1 kHz = 2 s.
const N: u32 = 2000;

/// Run the full gated calibration. Blocks until the capture completes
/// and then returns.
pub async fn apply() {
    let mut rcv = RAW_MULTI_IMU_DATA[0].receiver();

    ulog::log("[cal] waiting for drone to be stationary and level...");

    // Seed the LPFs with the first sample so they don't spend time ramping
    // up from zero (which would keep |acc_z - gravity| large for the first
    // ~50 ms and potentially be read as motion).
    let first = rcv.changed().await;
    let mut gyr_lpf = first.gyr;
    let mut acc_lpf = first.acc;

    // Phase 1: wait for filtered stationary+level to hold continuously.
    let mut ready_since: Option<Instant> = None;
    let mut last_heartbeat = Instant::now();
    loop {
        let d = rcv.changed().await;
        for i in 0..3 {
            gyr_lpf[i] += LPF_ALPHA * (d.gyr[i] - gyr_lpf[i]);
            acc_lpf[i] += LPF_ALPHA * (d.acc[i] - acc_lpf[i]);
        }

        let still = gyr_lpf[0].abs() < STILL_GYR_RADS
            && gyr_lpf[1].abs() < STILL_GYR_RADS
            && gyr_lpf[2].abs() < STILL_GYR_RADS;
        let level = acc_lpf[0].abs() < LEVEL_ACC_MS2 && acc_lpf[1].abs() < LEVEL_ACC_MS2;

        if still && level {
            if ready_since
                .map(|t| t.elapsed() >= Duration::from_millis(READY_HOLD_MS))
                .unwrap_or(false)
            {
                break;
            }
            if ready_since.is_none() {
                ready_since = Some(Instant::now());
            }
        } else {
            ready_since = None;
        }

        if last_heartbeat.elapsed() >= Duration::from_millis(HEARTBEAT_MS) {
            last_heartbeat = Instant::now();
            let status = if still && level {
                "OK"
            } else if !still {
                "moving"
            } else {
                "tilted"
            };
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[cal] {} gyr=[{:.3},{:.3},{:.3}] ax={:.2} ay={:.2}",
                status, gyr_lpf[0], gyr_lpf[1], gyr_lpf[2], acc_lpf[0], acc_lpf[1]
            );
            ulog::log(s.as_str());
        }
    }

    // Phase 2: hands-clear delay.
    ulog::log("[cal] stationary -- hands off, capturing in 5s...");
    Timer::after(Duration::from_millis(CLEAR_HANDS_MS)).await;

    // Phase 3: capture bias.
    ulog::log("[cal] capturing (2s)...");
    let mut sum_acc = [0.0_f64; 3];
    let mut sum_gyr = [0.0_f64; 3];
    for _ in 0..N {
        let d = rcv.changed().await;
        sum_acc[0] += d.acc[0] as f64;
        sum_acc[1] += d.acc[1] as f64;
        sum_acc[2] += d.acc[2] as f64;
        sum_gyr[0] += d.gyr[0] as f64;
        sum_gyr[1] += d.gyr[1] as f64;
        sum_gyr[2] += d.gyr[2] as f64;
    }

    let n = N as f64;
    let acc_bias = [
        (sum_acc[0] / n) as f32,
        (sum_acc[1] / n) as f32,
        // NED body frame (post override_imu_rot): level stationary drone
        // reads az = -GRAVITY, so bias = mean - (-GRAVITY) = mean + GRAVITY.
        // See D000066 where the old `mean - GRAVITY` formula (from the
        // pre-rotation chip-Z-up frame) yielded bias ~ -2g and sign-flipped
        // Z in the CAL output.
        (sum_acc[2] / n) as f32 + common::consts::GRAVITY,
    ];
    let gyr_bias = [
        (sum_gyr[0] / n) as f32,
        (sum_gyr[1] / n) as f32,
        (sum_gyr[2] / n) as f32,
    ];

    {
        let mut p = imu_reader::params::TABLE.params.write().await;
        p.cal_acc.bias = acc_bias;
        p.cal_gyr.bias = gyr_bias;
        // Scale is identity now that override_imu_rot handles chip->drone
        // axis conversion via rotation. Old [1,-1,-1] was a partial
        // workaround for the missing rotation.
        p.cal_acc.scale = [1.0, 1.0, 1.0];
        p.cal_gyr.scale = [1.0, 1.0, 1.0];
    }

    imu_reader::CHANNEL[0]
        .sender()
        .send(imu_reader::Message::ReloadParams)
        .await;

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[cal] acc=[{:.3},{:.3},{:.3}]",
        acc_bias[0], acc_bias[1], acc_bias[2]
    );
    ulog::log(s.as_str());

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[cal] gyr=[{:.4},{:.4},{:.4}]",
        gyr_bias[0], gyr_bias[1], gyr_bias[2]
    );
    ulog::log(s.as_str());

    CAL_DONE.store(true, Ordering::Relaxed);
}
