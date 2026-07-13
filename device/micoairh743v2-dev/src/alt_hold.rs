//! Altitude hold using MTF-01 lidar (primary) with DPS310 baro fallback.
//!
//! The MTF-01 lidar gives 0-8m AGL altitude at ~50 Hz with ~2cm accuracy,
//! far superior to the barometer which is corrupted by prop wash. The baro
//! is kept as a fallback and for altitude above 8m.

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

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
/// `flight.rs` checks the same flag for `TRUE_RATE_SP` so all
/// auto-control writes are silenced together. Updated per-tick by
/// `mission_fsm_task` to `matches!(state, State::Manual)`.
pub static MANUAL_BYPASS: AtomicBool = AtomicBool::new(false);

/// Set by the FSM while Auto runs the throttle as DIRECT thrust (the pilot
/// owns vertical, like Manual) instead of a position-based altitude command.
/// When set, alt_hold still computes normally -- so it keeps tracking baro for
/// `filtered_altitude`/`vertical_speed` and a clean AutoLand handoff -- but it
/// does NOT drive TRUE_Z_THRUST_SP; the FSM sends thrust directly. Unlike
/// MANUAL_BYPASS this does NOT disable the self-level angle controller, so Auto
/// keeps its attitude + GPS-lateral hold. Note: while set, the baro hard
/// ceiling is not auto-enforced -- the pilot owns altitude.
pub static AUTO_DIRECT_THRUST: AtomicBool = AtomicBool::new(false);

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

/// Hard altitude ceiling, runtime-settable. Default 2.0 m (indoor-safe).
/// Set via `set_ceiling` from `ceiling_mode` based on the TX15 SC switch:
/// SC=Low keeps 2.0; SC=Mid/High switches to 110.0 (EU Open A3 legal cap
/// of 120 m minus 10 m headroom for baro drift / EWMA lag).
///
/// Default-safe rationale: if the RC link never comes up, the ceiling
/// stays at 2.0 and the drone cannot legally fly outdoors. SC must be
/// actively raised by the pilot to unlock outdoor altitude.
///
/// D000009 incident (2026-04-22): the MTF-01 lidar got stuck reporting
/// ~0.05 m while the drone physically rocketed past ceiling height. Since
/// alt_hold only consulted baro as a lidar-miss fallback, the stuck-but-
/// reporting lidar silently locked the PID into max-thrust and there was
/// no independent safety net. Baro is now read every cycle and if it ever
/// shows the drone above the active ceiling, the controller forces
/// descent regardless of what lidar says. The check uses an EWMA over
/// raw baro so a single propwash spike cannot trip it.
static ACTIVE_CEILING_M_BITS: AtomicU32 = AtomicU32::new(0x4000_0000); // 2.0_f32.to_bits()

pub fn ceiling() -> f32 {
    f32::from_bits(ACTIVE_CEILING_M_BITS.load(Ordering::Relaxed))
}

pub fn set_ceiling(m: f32) {
    ACTIVE_CEILING_M_BITS.store(m.to_bits(), Ordering::Relaxed);
}

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

/// Boot-tick (ms, truncated to u32) of the most recent valid MTF-01 lidar
/// frame, stamped by `note_lidar_frame` from the reader task. The u32 ms
/// counter wraps after ~49 days of uptime, which a flight never reaches;
/// `wrapping_sub` in `lidar_is_live` keeps the delta correct across a wrap
/// regardless. 0 = no frame seen yet. Read by `lidar_is_live` to auto-detect
/// a missing/unplugged module.
static LIDAR_LAST_FRAME_MS: AtomicU32 = AtomicU32::new(0);

/// How long without a valid lidar frame before the MTF-01 is treated as
/// absent. The module streams at ~50 Hz, so 1 s is ~50 missed frames --
/// far beyond any quality-glitch dropout, but quick to react to an unplugged
/// or dead module.
const LIDAR_LIVE_TIMEOUT_MS: u32 = 1000;

/// Stamp the arrival of a valid lidar frame. Called by the MTF-01 reader.
pub fn note_lidar_frame() {
    LIDAR_LAST_FRAME_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
}

/// True while the MTF-01 lidar is actively streaming (a valid frame arrived
/// within LIDAR_LIVE_TIMEOUT_MS). False if the module is unplugged, dead, or
/// has not booted yet -- in which case LIDAR_ALT_M never updates and every
/// lidar-gated check would otherwise misfire (the FSM ground gate would block
/// arming forever, and alt_hold would trap itself in the indoor lidar-only
/// branch with no altitude source).
///
/// This auto-detects rather than relying on a hand-set flag: pull the module
/// and the firmware degrades to baro-only altitude + lidar-gate bypass on its
/// own; plug a working one back in and normal behavior resumes within ~1 s,
/// no recompile. A half-dead sensor that keeps emitting valid frames is NOT
/// covered -- unplug it rather than leaving it on the bus.
pub fn lidar_is_live() -> bool {
    let last = LIDAR_LAST_FRAME_MS.load(Ordering::Relaxed);
    last != 0 && (Instant::now().as_millis() as u32).wrapping_sub(last) < LIDAR_LIVE_TIMEOUT_MS
}

/// Filtered vertical speed (m/s, +climb / -descend), f32 bits in a u32, the
/// same trick as ACTIVE_CEILING_M_BITS. Published every control cycle so the
/// FSM can detect touchdown (descent stalls to ~0) without a lidar -- the
/// only ground-contact signal still available once the MTF-01 is gone, since
/// low commanded thrust looks identical whether descending or already landed.
static VERTICAL_SPEED_BITS: AtomicU32 = AtomicU32::new(0); // 0.0_f32.to_bits()

/// Latest filtered vertical speed (m/s, + = climbing). Derived from whatever
/// altitude source alt_hold is using (lidar or baro). 0.0 before alt_hold
/// takes over thrust.
pub fn vertical_speed() -> f32 {
    f32::from_bits(VERTICAL_SPEED_BITS.load(Ordering::Relaxed))
}

/// EWMA-filtered altitude (m), f32 bits in a u32. Published every control
/// cycle so the FSM can seed an AutoLand descent (and shadow the setpoint
/// during direct-thrust Auto) at the real current altitude instead of a stale
/// or floored value, which would otherwise cut thrust mid-air on handoff.
static FILTERED_ALT_BITS: AtomicU32 = AtomicU32::new(0); // 0.0_f32.to_bits()

/// Latest EWMA-filtered altitude (m) from alt_hold's active source (lidar or
/// baro). 0.0 before alt_hold takes over thrust.
pub fn filtered_altitude() -> f32 {
    f32::from_bits(FILTERED_ALT_BITS.load(Ordering::Relaxed))
}

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

/// Ground-idle gate (DJI takeoff model). When the commanded altitude is
/// essentially the floor AND the drone is physically near the ground, hold
/// motors at idle (0 thrust) instead of the BASE_THRUST hover feed-forward.
/// Non-latching, so the moment the throttle commands a real altitude (setpoint
/// rises above SP) the PID engages and the drone lifts off; conversely pulling
/// the throttle to the bottom in flight descends (PID) and idles on touchdown.
/// This is what makes "auto-arm sits idle until you raise the throttle" work --
/// without it, alt_hold would feed-forward hover thrust the instant it armed.
const GROUND_IDLE_SP_M: f32 = 0.08;
const GROUND_IDLE_ALT_M: f32 = 0.20;

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

pub async fn main(i2c: impl I2c, addr: u8) -> ! {
    defmt::info!("[alt_hold] task started");

    // Live battery voltage from the EWMA-filtered monitor (10 Hz). Replaces
    // the previous one-shot boot read; voltage compensation now tracks the
    // battery as it drains over the flight rather than being frozen at
    // BASE_THRUST's calibration voltage.
    let mut battery_rcv = crate::battery::BATTERY_FILTERED_MV
        .receiver()
        .expect("BATTERY_FILTERED_MV receiver slot");

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
    // Separate EWMA over RAW baro for the hard-ceiling check. Independent
    // of source-selection (lidar/baro) so it remains a true independent
    // safety witness, but smoothed so a single propwash spike cannot trip
    // the forced-descent. Init from baseline (== 0 m above takeoff).
    let mut baro_ewma: f32 = 0.0;
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

    // Edge-tracker for lidar liveness so the operator gets one log line when
    // the MTF-01 drops off the bus (-> baro-only) and one when it comes back,
    // instead of silence. Seeded live so a boot with the module absent emits
    // the "lost" line on the first cycle.
    let mut lidar_live_prev = true;

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
            // Clamp to the active hard ceiling so no upstream caller can
            // request altitude above the legal cap. ceiling_mode swaps the
            // ceiling between 2.0 m (SC=Low, indoor) and 110.0 m (SC=Mid/High,
            // outdoor with 10 m margin from the 120 m EU Open A3 cap).
            setpoint_m = sp.min(ceiling());
        }

        // Recompute voltage compensation each tick from the live filtered
        // battery voltage. Thrust ~ V^2, so scale by (nominal/actual)^2.
        // Fallback to NOMINAL_MV if the battery monitor hasn't published
        // its first sample yet (first ~100 ms after boot). USB-power
        // bench testing also falls back to NOMINAL_MV so v_comp stays 1.0
        // instead of boosting 2.3x against an irrelevant 5 V reading.
        let battery_mv = battery_rcv.try_get().unwrap_or(NOMINAL_MV as u32);
        let v_actual = if crate::battery::is_usb_power_range(battery_mv) {
            NOMINAL_MV
        } else {
            (battery_mv as f32).clamp(10000.0, 20000.0)
        };
        let v_comp = (NOMINAL_MV / v_actual) * (NOMINAL_MV / v_actual);

        // Always read baro -- it is the independent safety witness for the
        // primary lidar measurement. D000009 showed that a "stuck but still
        // publishing" lidar could silently lock the PID into max thrust
        // because baro was only consulted on a lidar miss.
        let baro_alt = baro
            .read()
            .await
            .ok()
            .map(|d| pressure_to_altitude(d.pressure_pa, baseline_pa));

        // Indoors (SC=Low, ~2 m ceiling) the lidar's 0-8 m range covers the
        // whole flight envelope, so the barometer contributes nothing but
        // propwash noise: with props spinning near the ground it reads 1-2 m
        // high, which made the lidar-disagree cross-check trust the bad baro,
        // conclude the drone was already at the ceiling, and command idle
        // thrust -- no takeoff (D000490). So indoors we run lidar-only: no baro
        // ceiling breach and no baro measurement. The baro path stays for
        // outdoor flight, where lidar cannot reach the 100 m cap.
        //
        // With the MTF-01 not streaming (unplugged/dead) there is no lidar to
        // run the indoor-only branch on, so force the outdoor/baro path
        // regardless of the active ceiling -- otherwise a low (indoor) ceiling
        // would strand alt_hold with no altitude source at all.
        let lidar_live = lidar_is_live();
        if lidar_live != lidar_live_prev {
            crate::log::log(if lidar_live {
                "[alt] MTF-01 lidar streaming -- normal altitude source"
            } else {
                "[alt] MTF-01 lidar lost -- baro-only altitude, lidar gates bypassed"
            });
            lidar_live_prev = lidar_live;
        }
        let indoor = lidar_live && ceiling() < 10.0;

        // SAFETY: hard baro-based ceiling (OUTDOOR only -- see above). If the
        // drone is above the active ceiling regardless of what lidar says,
        // force a gentle descent and skip the normal PID for this cycle. EWMA
        // over raw baro so a single propwash spike cannot trip it.
        if !indoor {
            if let Some(b) = baro_alt {
                baro_ewma = EWMA_ALPHA * b + (1.0 - EWMA_ALPHA) * baro_ewma;
                if baro_ewma > ceiling() {
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[alt] CEILING BREACH baro={:.2}m cap={:.1}m -- forced descent",
                        baro_ewma,
                        ceiling()
                    );
                    crate::log::log(s.as_str());
                    if !MANUAL_BYPASS.load(Ordering::Relaxed)
                        && !AUTO_DIRECT_THRUST.load(Ordering::Relaxed)
                    {
                        snd_thrust.send(CEILING_EMERGENCY_THRUST);
                    }
                    Timer::after_millis((DT * 1000.0) as u64).await;
                    continue;
                }
            }
        }

        // Primary altitude from lidar (0-8 m range check).
        let lidar_valid = lidar_rcv
            .try_changed()
            .filter(|&l| (0.0..=8.0).contains(&l));

        // Measurement-source selection.
        let alt_raw = if indoor {
            // Lidar-only indoors. On a brief lidar dropout (quality glitch near
            // the ~0.1 m minimum range) hold the last filtered altitude rather
            // than falling back to the propwash-corrupted baro -- holding keeps
            // the control loop (and thus the motor-governor keepalive) alive.
            match lidar_valid {
                Some(lidar) => {
                    if !use_lidar {
                        crate::log::log("[alt] switching to lidar");
                        use_lidar = true;
                    }
                    lidar
                }
                None => alt_filtered,
            }
        } else {
            // Outdoor: lidar with a baro cross-check. Baro is the primary
            // source above the 8 m lidar range and the D000009 stuck-lidar
            // safety witness (baro notably higher than a "stuck" lidar).
            match (lidar_valid, baro_alt) {
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
            }
        };

        alt_filtered = EWMA_ALPHA * alt_raw + (1.0 - EWMA_ALPHA) * alt_filtered;
        FILTERED_ALT_BITS.store(alt_filtered.to_bits(), Ordering::Relaxed);

        let err = setpoint_m - alt_filtered;
        let vel = (alt_filtered - prev_alt) / DT; // positive = climbing
        prev_alt = alt_filtered;
        VERTICAL_SPEED_BITS.store(vel.to_bits(), Ordering::Relaxed);
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
                s,
                "[alt] LANDING RAMP start h={:.2}m anchor={:.3}",
                alt_filtered, anchor,
            );
            crate::log::log(s.as_str());
        }

        // Ground idle (non-latching) takes priority over the hover feed-forward:
        // when the throttle commands the floor and we are near the ground, idle.
        let ground_idle = setpoint_m < GROUND_IDLE_SP_M && alt_filtered < GROUND_IDLE_ALT_M;

        let thrust = if ground_idle {
            // Bleed the integrator while idling so it cannot wind up against the
            // floor error and blunt the next takeoff.
            integral = 0.0;
            0.0
        } else {
            match landing_state {
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
            }
        };
        // Upper clamp raised from 5.0 -> 10.0 after D000130: alt_hold output
        // was pinned at 4.96 with sp=1.0m and h=0.05m, so the drone could not
        // climb out of ground effect. Motors were at ~750 DShot (~37% of 2047)
        // so the ESC/mechanical headroom is large; 5.0 was an over-conservative
        // software cap from an era before the closed-loop was trusted. 10.0
        // is 2.2x BASE_THRUST, plenty for normal climb while still bounded
        // well below full motor authority.
        if !MANUAL_BYPASS.load(Ordering::Relaxed) && !AUTO_DIRECT_THRUST.load(Ordering::Relaxed) {
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
