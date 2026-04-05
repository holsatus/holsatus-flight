//! Altitude hold using MTF-01 lidar (primary) with DPS310 baro fallback.
//!
//! The MTF-01 lidar gives 0-8m AGL altitude at ~50 Hz with ~2cm accuracy,
//! far superior to the barometer which is corrupted by prop wash. The baro
//! is kept as a fallback and for altitude above 8m.

use core::fmt::Write;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use libm::powf;

use crate::dps310_i2c::Dps310I2c;

/// External setpoint written by the mission sequencer (metres above takeoff point).
pub static ALTITUDE_SETPOINT: Signal<CriticalSectionRawMutex, f32> = Signal::new();

/// Lidar distance in metres, published by the MTF-01 reader task at ~50 Hz.
/// Negative value or quality=0 means invalid reading.
pub static LIDAR_ALT_M: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();

/// Optical flow velocity in body frame [vx, vy] m/s, published by MTF-01 reader at ~50 Hz.
/// vx = forward (positive = drone moving forward), vy = rightward.
/// Used by the flow_hold controller for velocity damping.
pub static FLOW_VEL_MS: Watch<CriticalSectionRawMutex, [f32; 2], 2> = Watch::new();

/// Base collective thrust for hovering (dimensionless 0..1) at NOMINAL_MV.
/// Voltage compensation scales this so the physical thrust stays constant
/// regardless of battery charge state.
const BASE_THRUST: f32 = 4.50;

/// Nominal battery voltage (mV) at which BASE_THRUST was tuned.
/// 4S storage voltage = 15200 mV (3.80 V/cell).
const NOMINAL_MV: f32 = 15200.0;

/// PI gains -- tune after initial bench test.
const KP: f32 = 0.5;
const KI: f32 = 0.1;

/// Controller update period in seconds (10 Hz).
const DT: f32 = 0.1;

/// EWMA smoothing factor for altitude (0 < alpha <= 1).
/// alpha = 0.3 gives ~230 ms effective lag at 10 Hz, which is fast enough
/// for altitude hold while rejecting sensor noise and indoor pressure drafts.
const EWMA_ALPHA: f32 = 0.3;

/// Number of baro readings to average for the baseline pressure.
const BASELINE_SAMPLES: usize = 20;

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
    let mut setpoint_m = 0.0_f32;
    let mut log_counter = 0u32;
    let mut alt_filtered = 0.0_f32;
    let mut lidar_rcv = LIDAR_ALT_M.receiver().unwrap();
    let mut use_lidar = false;

    let mut snd_thrust = common::signals::TRUE_Z_THRUST_SP.sender();

    loop {
        // Pick up a new altitude setpoint if the mission sequencer sent one.
        if let Some(sp) = ALTITUDE_SETPOINT.try_take() {
            setpoint_m = sp;
        }

        // Prefer lidar if available and valid (0-8m range, positive value).
        let alt_raw = if let Some(lidar_m) = lidar_rcv.try_changed() {
            if lidar_m >= 0.0 && lidar_m <= 8.0 {
                if !use_lidar {
                    crate::log::log("[alt] switching to lidar");
                    use_lidar = true;
                }
                Some(lidar_m)
            } else {
                None
            }
        } else {
            None
        };

        // Fall back to baro if no lidar reading this cycle.
        let alt_raw = match alt_raw {
            Some(a) => a,
            None => {
                if let Ok(d) = baro.read().await {
                    pressure_to_altitude(d.pressure_pa, baseline_pa)
                } else {
                    Timer::after_millis((DT * 1000.0) as u64).await;
                    continue;
                }
            }
        };

        alt_filtered = EWMA_ALPHA * alt_raw + (1.0 - EWMA_ALPHA) * alt_filtered;

        let err = setpoint_m - alt_filtered;
        integral = (integral + err * DT).clamp(-3.0, 3.0);
        let thrust = (BASE_THRUST + KP * err + KI * integral) * v_comp;
        snd_thrust.send(thrust.clamp(0.0, 5.0));

        // Log to UART at 1 Hz (every 10 cycles at 10 Hz).
        log_counter += 1;
        if log_counter >= 10 {
            log_counter = 0;
            let src = if use_lidar { "lid" } else { "bar" };
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "[alt:{}] h={:.3}m sp={:.1}m thr={:.3}",
                src, alt_filtered, setpoint_m, thrust);
            crate::log::log(s.as_str());
        }

        Timer::after_millis((DT * 1000.0) as u64).await;
    }
}
