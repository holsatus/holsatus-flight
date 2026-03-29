//! Barometric altitude hold using a simple PI controller.
//!
//! Reads the DPS310 barometer at 10 Hz and writes a collective thrust setpoint to
//! `common::signals::TRUE_Z_THRUST_SP`.  The `ALTITUDE_SETPOINT` signal is written
//! by the mission sequencer to command a target altitude.
//!
//! Barometric formula used:
//!   alt_m = 44330 * (1 - (p / p0) ^ 0.1903)
//!
//! where `p0` is the baseline pressure measured at startup.

use core::fmt::Write;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use libm::powf;

use crate::dps310_i2c::{Dps310I2c, ADDR_SDO_LOW};

/// External setpoint written by the mission sequencer (metres above takeoff point).
pub static ALTITUDE_SETPOINT: Signal<CriticalSectionRawMutex, f32> = Signal::new();

/// Base collective thrust for hovering (dimensionless 0..1).
/// Start conservative -- too low means the drone won't lift; too high means
/// it jumps and flips. Tune by observing the motor speeds at arming and
/// comparing to the weight. Typical 250mm quads hover around 0.30-0.40.
const BASE_THRUST: f32 = 0.25;

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

pub async fn main(i2c: impl I2c) -> ! {
    defmt::info!("[alt_hold] task started");

    let mut baro = Dps310I2c::new(i2c, ADDR_SDO_LOW);

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

    let mut snd_thrust = common::signals::TRUE_Z_THRUST_SP.sender();

    loop {
        // Pick up a new altitude setpoint if the mission sequencer sent one.
        if let Some(sp) = ALTITUDE_SETPOINT.try_take() {
            setpoint_m = sp;
        }

        if let Ok(d) = baro.read().await {
            let alt_raw = pressure_to_altitude(d.pressure_pa, baseline_pa);
            alt_filtered = EWMA_ALPHA * alt_raw + (1.0 - EWMA_ALPHA) * alt_filtered;

            let err = setpoint_m - alt_filtered;
            integral = (integral + err * DT).clamp(-1.0, 1.0);
            let thrust = BASE_THRUST + KP * err + KI * integral;
            snd_thrust.send(thrust.clamp(0.0, 1.0));
            defmt::trace!(
                "[alt_hold] alt={} sp={} thrust={}",
                alt_filtered,
                setpoint_m,
                thrust
            );

            // Log to UART at 1 Hz (every 10 cycles at 10 Hz).
            log_counter += 1;
            if log_counter >= 10 {
                log_counter = 0;
                crate::log::log(format_baro_log(alt_filtered, setpoint_m, thrust).as_str());
            }
        }

        Timer::after_millis((DT * 1000.0) as u64).await;
    }
}
