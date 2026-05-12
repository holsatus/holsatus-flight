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

// --- USB-power detection (bench testing without LiPo) ---
// When no LiPo is connected and the FC is powered from the USB-UART
// adapter or the USB-C port, the FC's 5 V rail bleeds through to the
// battery sense circuit and PC0 reads ~4.8-5.2 V. Without this carve-out
// the tier classifier would call that voltage "DAMAGE" and spam the log
// continuously, drowning out the messages we actually care about during
// bench testing of the RC interaction.
//
// 6.0 V enter / 6.5 V exit gives clean hysteresis -- a 4S LiPo plugged
// in (~13-17 V) crosses 6.5 V cleanly during the EWMA ramp-up, and even
// a profoundly damaged 4S (2.0 V/cell = 8 V) sits well above USB range.
pub const USB_POWER_ENTER_MV: u32 = 6_000;
pub const USB_POWER_EXIT_MV: u32 = 6_500;

/// Returns true if `mv` is in the USB-power range. Use this from
/// consumers (e.g. alt_hold) to suppress voltage-compensation math
/// during bench testing where reading the bus voltage is meaningless.
pub fn is_usb_power_range(mv: u32) -> bool {
    mv < USB_POWER_ENTER_MV
}

/// Filtered battery voltage in mV. EWMA-smoothed at 10 Hz with alpha=0.2
/// (~800 ms effective settling). Seeded on the first sample so consumers
/// never see 0. Consumers read via `try_get`.
pub static BATTERY_FILTERED_MV: Watch<CriticalSectionRawMutex, u32, 4> = Watch::new();

/// Current battery tier, updated by the monitor task each cycle with the
/// EWMA-derived classification (hysteresis applied). FSM reads this to
/// gate arming and trigger forced-descent. Published alongside
/// BATTERY_FILTERED_MV so consumers can act on the classification
/// without re-implementing the threshold logic.
pub static BATTERY_TIER: Watch<CriticalSectionRawMutex, Tier, 4> = Watch::new();

/// True if this tier permits motors-spinning operation. USB_POWER (bench
/// testing) and HEALTHY both qualify. Anything below HEALTHY blocks the
/// preflight + arm gates and triggers in-flight forced descent.
pub fn tier_allows_flight(tier: Tier) -> bool {
    matches!(tier, Tier::Healthy | Tier::UsbPower)
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Tier {
    Healthy,
    LandNow,
    Critical,
    Damage,
    /// FC powered from USB without a LiPo connected. Detected by the
    /// battery sense reading inside `[USB_POWER_ENTER_MV..USB_POWER_EXIT_MV]`
    /// (~5 V). Bench-test state; not safety-relevant.
    UsbPower,
}

impl Tier {
    pub fn label(self) -> &'static str {
        match self {
            Tier::Healthy => "HEALTHY",
            Tier::LandNow => "LAND_NOW",
            Tier::Critical => "CRITICAL",
            Tier::Damage => "DAMAGE",
            Tier::UsbPower => "USB_POWER",
        }
    }

    /// Tiers that warrant routing transitions through the priority log
    /// channel (so they always reach the operator even under telemetry
    /// flood). USB power is informational and uses the regular channel.
    pub fn is_severe(self) -> bool {
        matches!(self, Tier::LandNow | Tier::Critical | Tier::Damage)
    }
}

/// Tier classification with hysteresis. Downgrades (to more severe tier)
/// use enter thresholds; upgrades (to less severe) require crossing the
/// higher exit thresholds. Damage tier is latched -- but UsbPower
/// overrides that latch because going from a degraded battery to USB
/// bench-testing is a deliberate operator action, not a sensor anomaly.
fn classify(mv: u32, current: Tier) -> Tier {
    // USB-power range always takes precedence: any read in [0, ENTER_MV)
    // is unambiguously bench-test power, so the LiPo thresholds below
    // shouldn't apply.
    if mv < USB_POWER_ENTER_MV {
        return Tier::UsbPower;
    }
    // Exiting UsbPower requires crossing the higher hysteresis threshold,
    // so the EWMA ramp from ~5 V to ~15 V on LiPo plug-in doesn't briefly
    // mis-classify as Damage on the way past 13.2 V.
    if matches!(current, Tier::UsbPower) && mv < USB_POWER_EXIT_MV {
        return Tier::UsbPower;
    }
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
        Tier::Healthy | Tier::UsbPower => Tier::Healthy,
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
    let snd_tier = BATTERY_TIER.sender();

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
    snd_tier.send(tier);
    if !tier_allows_flight(tier) {
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
            // Severe tiers go through the priority channel; UsbPower and
            // Healthy transitions are informational.
            if new_tier.is_severe() || tier.is_severe() {
                ulog::log_critical(s.as_str()).await;
            } else {
                ulog::log(s.as_str());
            }
            tier = new_tier;
            snd_tier.send(tier);
        }

        if last_periodic_log.elapsed().as_millis() as u64 >= PERIODIC_LOG_MS {
            last_periodic_log = embassy_time::Instant::now();
            let mut s: String<48> = String::new();
            let _ = write!(s, "[bat] {} mV ({})", filtered_mv, tier.label());
            ulog::log(s.as_str());
        }
    }
}

