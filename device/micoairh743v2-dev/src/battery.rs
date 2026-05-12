//! Continuous battery voltage monitoring with EWMA filtering and tiered
//! threshold-transition logging. Phase 1: monitoring only -- no flight
//! behavior changes. The logged data shows how voltage actually behaves
//! under flight load, which informs the Phase 2 thresholds for graceful
//! action.
//!
//! ADC1 owned by this task. Replaces the previous one-shot boot read in
//! `read_battery_mv()`; alt_hold's voltage compensation now follows the
//! filtered value in real time instead of being frozen at boot.

use core::fmt::Write as _;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use heapless::String;

use crate::log as ulog;
use crate::resources::BatteryResources;

/// Voltage-divider ratio on PC0 (R1 + R2 / R2). Empirically calibrated.
const V_DIV: u32 = 21;
const ADC_FULL: u32 = 65535;
const VREF_MV: u32 = 3300;

// --- 4S LiPo tiered thresholds (open-circuit reference) ---
// Per-cell × 4 cells:
//   3.5 V/cell = 14_000 mV   LAND_NOW soft warning
//   3.4 V/cell = 13_600 mV   CRITICAL, descend immediately
//   3.3 V/cell = 13_200 mV   DAMAGE, cells degrade
//
// Hysteresis: separate enter/exit so a transient sag from a throttle punch
// doesn't bounce us in and out of a tier. EWMA does most of the smoothing
// (~800 ms effective settling); hysteresis is belt-and-suspenders.
pub const LAND_NOW_ENTER_MV: u32 = 14_000;
pub const LAND_NOW_EXIT_MV: u32 = 14_400;
pub const CRITICAL_ENTER_MV: u32 = 13_600;
pub const CRITICAL_EXIT_MV: u32 = 13_900;
pub const DAMAGE_ENTER_MV: u32 = 13_200;
// Damage tier is latched -- once entered, stays until reboot. A cell
// reaching the damage threshold should be charged before more discharge.

/// Filtered battery voltage in mV. EWMA-smoothed at 10 Hz with alpha=0.2
/// (~800 ms effective settling). Seeded on the first sample so consumers
/// never see 0. Consumers read via `try_get`.
pub static BATTERY_FILTERED_MV: Watch<CriticalSectionRawMutex, u32, 4> = Watch::new();

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Tier {
    Healthy,
    LandNow,
    Critical,
    Damage,
}

impl Tier {
    pub fn label(self) -> &'static str {
        match self {
            Tier::Healthy => "HEALTHY",
            Tier::LandNow => "LAND_NOW",
            Tier::Critical => "CRITICAL",
            Tier::Damage => "DAMAGE",
        }
    }
}

/// Tier classification with hysteresis. Downgrades (to more severe tier)
/// use enter thresholds; upgrades (to less severe) require crossing the
/// higher exit thresholds. Damage tier is latched.
fn classify(mv: u32, current: Tier) -> Tier {
    if mv < DAMAGE_ENTER_MV {
        return Tier::Damage;
    }
    if mv < CRITICAL_ENTER_MV {
        return Tier::Critical;
    }
    if mv < LAND_NOW_ENTER_MV {
        return Tier::LandNow;
    }
    match current {
        Tier::Damage => Tier::Damage,
        Tier::Critical => {
            if mv >= LAND_NOW_EXIT_MV {
                Tier::Healthy
            } else if mv >= CRITICAL_EXIT_MV {
                Tier::LandNow
            } else {
                Tier::Critical
            }
        }
        Tier::LandNow => {
            if mv >= LAND_NOW_EXIT_MV {
                Tier::Healthy
            } else {
                Tier::LandNow
            }
        }
        Tier::Healthy => Tier::Healthy,
    }
}

fn raw_to_mv(raw: u16) -> u32 {
    (raw as u32 * VREF_MV * V_DIV) / ADC_FULL
}

#[embassy_executor::task]
pub async fn battery_monitor_task(r: BatteryResources) -> ! {
    let mut adc = Adc::new(r.adc);
    let mut pin = r.pin_v;
    let snd = BATTERY_FILTERED_MV.sender();

    // First sample seeds the EWMA so the filter doesn't ramp from zero.
    let raw = adc.blocking_read(&mut pin, SampleTime::CYCLES64_5);
    let mut filtered_mv = raw_to_mv(raw);
    snd.send(filtered_mv);
    {
        let mut s: String<48> = String::new();
        let _ = write!(s, "[bat] boot voltage={} mV", filtered_mv);
        ulog::log(s.as_str());
    }

    let mut tier = classify(filtered_mv, Tier::Healthy);
    if tier != Tier::Healthy {
        let mut s: String<64> = String::new();
        let _ = write!(s, "[bat] boot tier={} at {} mV", tier.label(), filtered_mv);
        ulog::log_critical(s.as_str()).await;
    }

    // EWMA: filtered = (1 - alpha) * filtered + alpha * raw, alpha = 1/5 = 0.2.
    // ~80% settling in ~9 samples = 900 ms at 10 Hz sampling. Long enough to
    // smooth burst current sag during stick punches, short enough to catch
    // real depletion within ~1 s.
    const ALPHA_NUM: u32 = 1;
    const ALPHA_DEN: u32 = 5;

    const SAMPLE_PERIOD_MS: u64 = 100;
    const PERIODIC_LOG_MS: u64 = 2_000;
    let mut last_periodic_log = embassy_time::Instant::now();

    loop {
        Timer::after_millis(SAMPLE_PERIOD_MS).await;

        let raw = adc.blocking_read(&mut pin, SampleTime::CYCLES64_5);
        let raw_mv = raw_to_mv(raw);
        filtered_mv = (filtered_mv * (ALPHA_DEN - ALPHA_NUM) + raw_mv * ALPHA_NUM) / ALPHA_DEN;
        snd.send(filtered_mv);

        let new_tier = classify(filtered_mv, tier);
        if new_tier != tier {
            let mut s: String<64> = String::new();
            let _ = write!(
                s,
                "[bat] {} -> {} at {} mV (raw={})",
                tier.label(),
                new_tier.label(),
                filtered_mv,
                raw_mv,
            );
            ulog::log_critical(s.as_str()).await;
            tier = new_tier;
        }

        if last_periodic_log.elapsed().as_millis() as u64 >= PERIODIC_LOG_MS {
            last_periodic_log = embassy_time::Instant::now();
            let mut s: String<48> = String::new();
            let _ = write!(s, "[bat] {} mV ({})", filtered_mv, tier.label());
            ulog::log(s.as_str());
        }
    }
}

