//! Altitude hold using MTF-01 lidar (primary) with DPS310 baro fallback.
//!
//! The MTF-01 lidar gives 0-8m AGL altitude at ~50 Hz with ~2cm accuracy,
//! far superior to the barometer which is corrupted by prop wash. The baro
//! is kept as a fallback and for altitude above 8m.

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Instant, Timer};
use embedded_hal_async::i2c::I2c;
use libm::powf;

use crate::dps310_i2c::Dps310I2c;

/// External setpoint written by the mission sequencer (metres above takeoff point).
pub static ALTITUDE_SETPOINT: Signal<CriticalSectionRawMutex, f32> = Signal::new();

/// When true, alt_hold's PID skips its `TRUE_Z_THRUST_SP` write so the
/// FSM's Manual (ACRO) state can own thrust from the throttle stick
/// without racing this task. The `angle_to_rate_bridge` in
/// `free_test.rs` checks the same flag for `TRUE_RATE_SP` so all
/// auto-control writes are silenced together. Updated per-tick by
/// `mission_fsm_task` to `matches!(state, State::Manual)`.
pub static MANUAL_BYPASS: AtomicBool = AtomicBool::new(false);

/// Lidar distance in metres, published by the MTF-01 reader task at ~50 Hz.
/// Negative value or quality=0 means invalid reading.
pub static LIDAR_ALT_M: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();

/// Optical flow velocity in body frame [vx, vy] m/s, published by MTF-01 reader at ~50 Hz.
/// vx = forward (positive = drone moving forward), vy = rightward.
/// Used by the flow_hold controller for velocity damping.
pub static FLOW_VEL_MS: Watch<CriticalSectionRawMutex, [f32; 2], 2> = Watch::new();

/// Base collective thrust for hovering (dimensionless) at NOMINAL_MV.
/// Voltage compensation scales this so the physical thrust stays constant
/// regardless of battery charge state.
///
/// Calibration history: 4.50 -> 6.00 -> 7.50 -> 8.50 -> 8.00. Each upward
/// step was driven by a run where the drone got stuck in ground effect
/// (D000130, D000134, D000136) or descended with controller saturated
/// (D000004). 8.50 was an over-correction: D000009 rocketed to the
/// ceiling because of a stuck lidar, and D000018 showed the baseline
/// itself is well above hover -- at BASE=8.5 with v_comp~0.918 the
/// err=0 output is 7.80 post-v_comp, which is ~10% above the ~7.0-7.3
/// post-v_comp needed for out-of-ground-effect hover. Net: drone
/// continuously accelerates upward and the D-term cannot pull it back
/// below hover. At 8.00 the err=0 baseline becomes 7.34 -- closer to
/// hover, small climb authority from PID alone, drone should track the
/// setpoint ramp instead of outrunning it.
const BASE_THRUST: f32 = 8.00;

/// Nominal battery voltage (mV) at which BASE_THRUST was tuned.
/// 4S storage voltage = 15200 mV (3.80 V/cell).
const NOMINAL_MV: f32 = 15200.0;

/// PID gains -- KD damps vertical velocity to prevent overshoot at
/// liftoff and during setpoint transitions. Validated in sim first.
const KP: f32 = 0.5;
const KI: f32 = 0.1;
const KD: f32 = 0.8;

/// Controller update period in seconds (10 Hz).
const DT: f32 = 0.1;

/// EWMA smoothing factor for altitude (0 < alpha <= 1).
/// alpha = 0.3 gives ~230 ms effective lag at 10 Hz, which is fast enough
/// for altitude hold while rejecting sensor noise and indoor pressure drafts.
const EWMA_ALPHA: f32 = 0.3;

/// Hard altitude ceiling (m, above baseline). Baro-based safety net.
/// D000009 incident (2026-04-22): the MTF-01 lidar got stuck reporting
/// ~0.05 m while the drone physically rocketed past ceiling height. Since
/// alt_hold only consulted baro as a lidar-miss fallback, the stuck-but-
/// reporting lidar silently locked the PID into max-thrust and there was
/// no independent safety net. Baro is now read every cycle and if it ever
/// shows the drone above HARD_CEILING_M, the controller forces descent
/// regardless of what lidar says.
///
/// Sized to 2.0 m instead of sp+0.5 because baro propwash noise on a quad
/// is ~+/-0.3 m during hover (and spikes to ~+/-0.5 m). At setpoint 1.0 m
/// a tighter ceiling (1.5 m) would false-trip on a propwash peak. 2.0 m
/// gives ~3 sigma of propwash headroom while still well below a typical
/// room ceiling (~2.4-2.6 m).
const HARD_CEILING_M: f32 = 2.0;

/// Post-v_comp thrust used during a HARD_CEILING_M breach -- well below
/// typical hover thrust so the drone sinks at a bounded terminal rate
/// instead of falling free. With BASE_THRUST ~= 8.5 at baseline, 4.0 is
/// about half of hover thrust, giving a gentle but decisive descent.
const CEILING_EMERGENCY_THRUST: f32 = 4.0;

/// If baro says the drone is more than this far above what lidar reports,
/// assume lidar is lying (D000009 failure mode: stuck at 0.05 m while drone
/// climbing). Switch the PID measurement source to baro rather than
/// believing the lidar. 0.5 m is larger than typical baro propwash noise
/// (~0.1-0.3 m) but small enough to catch lidar-stuck-at-zero cases early.
const LIDAR_DISAGREE_M: f32 = 0.5;

/// Number of baro readings to average for the baseline pressure.
const BASELINE_SAMPLES: usize = 20;

/// Altitude the drone must have cleared at least once before landing-mode
/// can arm. Prevents the trigger from firing during the P1 takeoff ramp
/// (alt and setpoint both start <0.3 m on the pad).
const LANDING_ARM_M: f32 = 0.5;

/// Enter landing mode when setpoint drops below this AND altitude is
/// also below LANDING_TRIGGER_ALT_M (mission is committed to landing
/// AND we are already close to the ground).
const LANDING_TRIGGER_SP_M: f32 = 0.20;
const LANDING_TRIGGER_ALT_M: f32 = 0.35;

/// Duration of the open-loop thrust ramp, seconds. Thrust goes from the
/// anchor captured at entry to zero over this window.
/// Tuned observation D000105: drone takes ~0.8 s to traverse the last
/// ~0.3 m when PID is out of the way, so 0.8 s keeps the ramp finishing
/// roughly when the drone touches down. Too fast = hard landing; too
/// slow = PID-like "hover in ground effect forever" behaviour.
const LANDING_DURATION_S: f32 = 0.8;

fn pressure_to_altitude(p: f32, p0: f32) -> f32 {
    44330.0 * (1.0 - powf(p / p0, 0.1903))
}

async fn sample_baseline(baro: &mut Dps310I2c<impl I2c>) -> f32 {
    let mut sum = 0.0_f32;
    let mut count = 0usize;
    while count < BASELINE_SAMPLES {
        if let Ok(d) = baro.read().await {
            sum += d.pressure_pa;
            count += 1;
        }
        Timer::after_millis(100).await;
    }
    sum / BASELINE_SAMPLES as f32
}

fn format_baro_log(alt_m: f32, setpoint_m: f32, thrust: f32) -> heapless::String<64> {
    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[baro] alt={:.3}m sp={:.1}m thr={:.3}",
        alt_m, setpoint_m, thrust
    );
    s
}

pub async fn main(i2c: impl I2c, addr: u8, battery_mv: u32) -> ! {
    defmt::info!("[alt_hold] task started");

    // Voltage compensation: thrust ~ V^2, so scale by (nominal/actual)^2.
    // Clamp to avoid division by zero or extreme values.
    let v_actual = (battery_mv as f32).clamp(10000.0, 20000.0);
    let v_comp = (NOMINAL_MV / v_actual) * (NOMINAL_MV / v_actual);

    let mut baro = Dps310I2c::new(i2c, addr);

    loop {
        match baro.init().await {
            Ok(()) => {
                defmt::info!("[alt_hold] DPS310 initialized");
                crate::log::log("[baro] DPS310 init OK");
                crate::log::SENSORS_READY.fetch_or(
                    crate::log::SENSOR_BARO_BIT,
                    core::sync::atomic::Ordering::Release,
                );
                break;
            }
            Err(e) => {
                defmt::error!("[alt_hold] DPS310 init failed: {:?}", e);
                crate::log::log("[baro] DPS310 init FAIL");
                Timer::after_secs(1).await;
            }
        }
    }

    defmt::info!(
        "[alt_hold] sampling baseline pressure ({} readings)...",
        BASELINE_SAMPLES
    );
    let baseline_pa = sample_baseline(&mut baro).await;
    defmt::info!("[alt_hold] baseline pressure: {} Pa", baseline_pa);

    let mut integral = 0.0_f32;
    let mut prev_alt = 0.0_f32;
    let mut log_counter = 0u32;
    let mut alt_filtered = 0.0_f32;
    let mut lidar_rcv = LIDAR_ALT_M.receiver().unwrap();
    let mut use_lidar = false;

    // Landing-ramp state. The PID cannot cleanly land on this airframe:
    // at low altitude ground effect lifts the drone just as the KD term
    // (designed to damp liftoff overshoot) adds thrust in response to
    // descent velocity, so every flight in D000087-D000105 ended with a
    // bounce at 0.15-0.20 m. Once we are committed to landing (mission
    // setpoint near zero AND we are already near the ground), stop
    // running the PID and ramp thrust linearly to zero over
    // LANDING_DURATION_S. Sticky once entered.
    let mut landing_armed = false;
    let mut landing_state: Option<(Instant, f32)> = None;

    let mut snd_thrust = common::signals::TRUE_Z_THRUST_SP.sender();

    // Wait for the mission sequencer to hand over altitude control by
    // signalling the first ALTITUDE_SETPOINT. Before that point, the mission
    // owns TRUE_Z_THRUST_SP directly (P0 thrust ramp), and if we ran our
    // 10 Hz control loop concurrently we would overwrite every 10th ramp
    // sample with our own BASE_THRUST estimate. That race produced the
    // 10 Hz motor-thrust jitter visible in D000005/000001.LOG during the
    // 10 s ramp.
    crate::log::log("[alt] waiting for mission setpoint...");
    let mut setpoint_m: f32 = ALTITUDE_SETPOINT.wait().await;
    crate::log::log("[alt] mission setpoint received, taking over thrust");

    loop {
        // Pick up a new altitude setpoint if the mission sequencer sent one.
        if let Some(sp) = ALTITUDE_SETPOINT.try_take() {
            setpoint_m = sp;
        }

        // Always read baro -- it is the independent safety witness for the
        // primary lidar measurement. D000009 showed that a "stuck but still
        // publishing" lidar could silently lock the PID into max thrust
        // because baro was only consulted on a lidar miss.
        let baro_alt = baro
            .read()
            .await
            .ok()
            .map(|d| pressure_to_altitude(d.pressure_pa, baseline_pa));

        // SAFETY: hard baro-based ceiling. If the drone is above
        // HARD_CEILING_M regardless of what lidar says, force a gentle
        // descent and skip the normal PID for this cycle. The drone will
        // keep sinking until baro reads below the ceiling, at which point
        // normal control resumes.
        if let Some(b) = baro_alt {
            if b > HARD_CEILING_M {
                let mut s: heapless::String<64> = heapless::String::new();
                let _ = write!(
                    s, "[alt] CEILING BREACH baro={:.2}m -- forced descent", b
                );
                crate::log::log(s.as_str());
                if !MANUAL_BYPASS.load(Ordering::Relaxed) {
                    snd_thrust.send(CEILING_EMERGENCY_THRUST);
                }
                Timer::after_millis((DT * 1000.0) as u64).await;
                continue;
            }
        }

        // Primary altitude from lidar (0-8 m range check).
        let lidar_valid = lidar_rcv
            .try_changed()
            .filter(|&l| (0.0..=8.0).contains(&l));

        // Measurement-source selection with a baro cross-check:
        //   - lidar + baro agree (or baro not available): use lidar
        //   - baro is notably higher than lidar (D000009 failure mode):
        //     switch to baro as the measurement source
        //   - lidar missing this cycle: fall back to baro
        //   - neither source: sleep and retry
        let alt_raw = match (lidar_valid, baro_alt) {
            (Some(lidar), Some(baro)) if baro > lidar + LIDAR_DISAGREE_M => {
                if use_lidar {
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[alt] LIDAR DISAGREE lid={:.2} bar={:.2} -- using baro",
                        lidar, baro
                    );
                    crate::log::log(s.as_str());
                    use_lidar = false;
                }
                baro
            }
            (Some(lidar), _) => {
                if !use_lidar {
                    crate::log::log("[alt] switching to lidar");
                    use_lidar = true;
                }
                lidar
            }
            (None, Some(baro)) => {
                if use_lidar {
                    crate::log::log("[alt] lidar lost -- using baro");
                    use_lidar = false;
                }
                baro
            }
            (None, None) => {
                Timer::after_millis((DT * 1000.0) as u64).await;
                continue;
            }
        };

        alt_filtered = EWMA_ALPHA * alt_raw + (1.0 - EWMA_ALPHA) * alt_filtered;

        let err = setpoint_m - alt_filtered;
        let vel = (alt_filtered - prev_alt) / DT; // positive = climbing
        prev_alt = alt_filtered;
        // Integral clamp tightened 3.0 -> 1.0 after D000020: the drone
        // integrated a full 6 s of err>0 during P1 climb, saturated the
        // +/-3.0 window, then overshot by 0.22 m AND couldn't unwind fast
        // enough to command a clean descent (at err=-0.2 the integrator
        // bleeds off at just 0.02/cycle, ~15 s to unwind). With +/-1.0
        // the max integral contribution is 0.1 pre-v_comp (instead of 0.3),
        // overshoot shrinks proportionally, and unwind is 3x faster in
        // the same time-scale -- so descent becomes responsive rather than
        // the current "thrust slowly fades for 15 s" behaviour.
        integral = (integral + err * DT).clamp(-1.0, 1.0);
        // D-term damps vertical velocity: climbing too fast reduces thrust,
        // descending too fast increases thrust. Prevents overshoot at liftoff.
        let pid_thrust = (BASE_THRUST + KP * err + KI * integral - KD * vel) * v_comp;

        // Arm landing-mode once the drone has actually flown. Without this
        // guard the trigger below would fire during P1 takeoff where alt
        // and setpoint are both near zero.
        if alt_filtered > LANDING_ARM_M {
            landing_armed = true;
        }

        // Latch into landing mode on the first cycle where the mission has
        // commanded a low setpoint AND we are close to ground. Anchor the
        // ramp to the PID's current output for a seamless switchover
        // (no step change when we swap out the controller). Cap the
        // anchor at hover so an above-hover PID spike at entry cannot
        // prolong the ramp.
        if landing_state.is_none()
            && landing_armed
            && setpoint_m < LANDING_TRIGGER_SP_M
            && alt_filtered < LANDING_TRIGGER_ALT_M
        {
            let anchor = pid_thrust.clamp(0.0, BASE_THRUST * v_comp);
            landing_state = Some((Instant::now(), anchor));
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s, "[alt] LANDING RAMP start h={:.2}m anchor={:.3}",
                alt_filtered, anchor,
            );
            crate::log::log(s.as_str());
        }

        let thrust = match landing_state {
            Some((start, anchor)) => {
                // Open-loop linear ramp to zero. Altitude-independent: if
                // lidar drops out mid-flare the drone still lands because
                // thrust is driven to zero regardless of what altitude
                // says. PID is bypassed entirely.
                let t = start.elapsed().as_millis() as f32 / 1000.0;
                let factor = (1.0 - t / LANDING_DURATION_S).clamp(0.0, 1.0);
                anchor * factor
            }
            None => pid_thrust,
        };
        // Upper clamp raised from 5.0 -> 10.0 after D000130: alt_hold output
        // was pinned at 4.96 with sp=1.0m and h=0.05m, so the drone could not
        // climb out of ground effect. Motors were at ~750 DShot (~37% of 2047)
        // so the ESC/mechanical headroom is large; 5.0 was an over-conservative
        // software cap from an era before the closed-loop was trusted. 10.0
        // is 2.2x BASE_THRUST, plenty for normal climb while still bounded
        // well below full motor authority.
        if !MANUAL_BYPASS.load(Ordering::Relaxed) {
            snd_thrust.send(thrust.clamp(0.0, 10.0));
        }

        // Log to UART at 5 Hz (every 2 cycles at 10 Hz).
        // Higher rate than before (was 1 Hz) to capture D-term dynamics.
        log_counter += 1;
        if log_counter >= 2 {
            log_counter = 0;
            let src = if use_lidar { "lid" } else { "bar" };
            // Include baro reading in every log line so the D000009-style
            // lidar-lying failure is visible at a glance in post-flight
            // analysis. baro=NaN if read failed this cycle.
            let baro_disp = baro_alt.unwrap_or(f32::NAN);
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[alt:{}] h={:.3}m sp={:.1}m v={:.2} baro={:.2}m thr={:.3}",
                src, alt_filtered, setpoint_m, vel, baro_disp, thrust
            );
            crate::log::log(s.as_str());
        }

        Timer::after_millis((DT * 1000.0) as u64).await;
    }
}
