//! Continuous battery voltage monitoring with EWMA filtering and tiered
//! threshold-transition logging. Detects 2S/3S/4S cell count at boot
//! from pack voltage and applies per-cell thresholds; once locked, cell
//! count never changes for the rest of the session so that voltage sag
//! during flight cannot reclassify the pack.
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

// --- Per-cell tiered thresholds (open-circuit reference) ---
//   3.5 V/cell   LAND_NOW soft warning
//   3.4 V/cell   CRITICAL, descend immediately
//   3.3 V/cell   DAMAGE, cells degrade
//
// Pack-level thresholds are these constants times the detected cell
// count (2/3/4). Hysteresis: separate enter/exit so a transient sag
// from a throttle punch doesn't bounce us in and out of a tier. EWMA
// does most of the smoothing (~800 ms effective settling); hysteresis
// is belt-and-suspenders.
pub const LAND_NOW_ENTER_PER_CELL_MV: u32 = 3_500;
pub const LAND_NOW_EXIT_PER_CELL_MV: u32 = 3_600;
pub const CRITICAL_ENTER_PER_CELL_MV: u32 = 3_400;
pub const CRITICAL_EXIT_PER_CELL_MV: u32 = 3_475;
pub const DAMAGE_ENTER_PER_CELL_MV: u32 = 3_300;
// Damage tier is latched -- once entered, stays until reboot. A cell
// reaching the damage threshold should be charged before more discharge.

// --- Cell-count detection ---
// Healthy pack voltage ranges, computed as
// [LAND_NOW_ENTER_PER_CELL_MV * cells, FULL_CHARGE_PER_CELL_MV * cells +
// DETECT_TOP_SLACK_MV]:
//   2S: 7.0 V - 8.5 V
//   3S: 10.5 V - 12.7 V
//   4S: 14.0 V - 16.9 V
// The gaps between ranges (8.5 - 10.5 V, 12.7 - 14.0 V) are unambiguous:
// no flyable pack of any supported size sits there, so we refuse to
// arm. The pathological case is a deeply-depleted 4S reading ~13 V
// that could resemble a 3S, but such a pack is below 3S LAND_NOW
// anyway (10.5 V) so the gap test still rejects it.
//
// DETECT_TOP_SLACK_MV gives 100 mV at the top to absorb fresh-off-
// charger surface voltage and ADC measurement noise so a freshly-
// charged pack isn't kept on the ground by a few mV of overshoot.
const FULL_CHARGE_PER_CELL_MV: u32 = 4_200;
const DETECT_TOP_SLACK_MV: u32 = 100;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum CellCount {
    S2,
    S3,
    S4,
}

impl CellCount {
    pub fn cells(self) -> u32 {
        match self {
            Self::S2 => 2,
            Self::S3 => 3,
            Self::S4 => 4,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::S2 => "2S",
            Self::S3 => "3S",
            Self::S4 => "4S",
        }
    }
}

fn detect_cell_count(mv: u32) -> Option<CellCount> {
    for cc in [CellCount::S2, CellCount::S3, CellCount::S4] {
        let n = cc.cells();
        let lo = n * LAND_NOW_ENTER_PER_CELL_MV;
        let hi = n * FULL_CHARGE_PER_CELL_MV + DETECT_TOP_SLACK_MV;
        if mv >= lo && mv <= hi {
            return Some(cc);
        }
    }
    None
}

// --- USB-power detection (bench testing without LiPo) ---
// When no LiPo is connected and the FC is powered from the USB-UART
// adapter or the USB-C port, the FC's 5 V rail bleeds through to the
// battery sense circuit and PC0 reads ~4.8-5.2 V. Without this carve-out
// the tier classifier would call that voltage "DAMAGE" and spam the log
// continuously, drowning out the messages we actually care about during
// bench testing of the RC interaction.
//
// 6.0 V enter / 6.5 V exit sits below the 2S LAND_NOW floor (7.0 V),
// so any flyable LiPo plugged in crosses out of USB range cleanly.
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
///
/// `cells = None` means cell count hasn't been locked yet (boot voltage
/// outside any valid pack range, e.g. UsbPower at boot or a pack in the
/// dead zone between 3S and 4S). In that state we refuse to fly: USB
/// range stays UsbPower, anything else is Damage until reboot.
fn classify(mv: u32, current: Tier, cells: Option<CellCount>) -> Tier {
    // USB-power range always takes precedence: any read in [0, ENTER_MV)
    // is unambiguously bench-test power, so the LiPo thresholds below
    // shouldn't apply.
    if mv < USB_POWER_ENTER_MV {
        return Tier::UsbPower;
    }
    // Exiting UsbPower requires crossing the higher hysteresis threshold.
    if matches!(current, Tier::UsbPower) && mv < USB_POWER_EXIT_MV {
        return Tier::UsbPower;
    }

    // No cell count locked: voltage doesn't match any supported pack
    // (e.g. deeply-depleted pack in the 8.5-10.5 V or 12.7-14.0 V gap).
    // Refuse flight.
    let Some(cc) = cells else {
        return Tier::Damage;
    };
    let n = cc.cells();
    let damage_enter = n * DAMAGE_ENTER_PER_CELL_MV;
    let critical_enter = n * CRITICAL_ENTER_PER_CELL_MV;
    let critical_exit = n * CRITICAL_EXIT_PER_CELL_MV;
    let land_now_enter = n * LAND_NOW_ENTER_PER_CELL_MV;
    let land_now_exit = n * LAND_NOW_EXIT_PER_CELL_MV;

    if mv < damage_enter {
        return Tier::Damage;
    }
    if mv < critical_enter {
        return Tier::Critical;
    }
    if mv < land_now_enter {
        return Tier::LandNow;
    }
    match current {
        Tier::Damage => Tier::Damage,
        Tier::Critical => {
            if mv >= land_now_exit {
                Tier::Healthy
            } else if mv >= critical_exit {
                Tier::LandNow
            } else {
                Tier::Critical
            }
        }
        Tier::LandNow => {
            if mv >= land_now_exit {
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

    // Detect cell count once from the seed reading (only if above USB
    // range -- USB power has no LiPo to classify). Once set, never
    // changes: in-flight voltage sag could otherwise drop a 4S into
    // the 3S range and silently widen the apparent margin.
    let mut cell_count: Option<CellCount> = if filtered_mv >= USB_POWER_EXIT_MV {
        detect_cell_count(filtered_mv)
    } else {
        None
    };
    {
        let mut s: String<64> = String::new();
        match cell_count {
            Some(cc) => {
                let _ = write!(s, "[bat] boot voltage={} mV pack={}", filtered_mv, cc.label());
            }
            None => {
                let _ = write!(s, "[bat] boot voltage={} mV pack=unknown", filtered_mv);
            }
        }
        ulog::log(s.as_str());
    }

    let mut tier = classify(filtered_mv, Tier::Healthy, cell_count);
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

        // Hot-plug guard: if a LiPo is plugged in while we're in
        // UsbPower, the EWMA would ramp from ~5 V slowly through the
        // 2S detection range (7-8.5 V) on its way to the real pack
        // voltage, mis-locking cell_count to 2S. Reset the filter to
        // the raw read at the moment we leave USB so detection runs
        // against true pack voltage.
        let exiting_usb = matches!(tier, Tier::UsbPower) && raw_mv >= USB_POWER_EXIT_MV;
        filtered_mv = if exiting_usb {
            raw_mv
        } else {
            (filtered_mv * (ALPHA_DEN - ALPHA_NUM) + raw_mv * ALPHA_NUM) / ALPHA_DEN
        };
        snd.send(filtered_mv);

        if cell_count.is_none() && filtered_mv >= USB_POWER_EXIT_MV {
            if let Some(cc) = detect_cell_count(filtered_mv) {
                cell_count = Some(cc);
                let mut s: String<64> = String::new();
                let _ = write!(s, "[bat] pack detected: {} at {} mV", cc.label(), filtered_mv);
                ulog::log(s.as_str());
            }
        }

        let new_tier = classify(filtered_mv, tier, cell_count);
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
            let mut s: String<64> = String::new();
            match cell_count {
                Some(cc) => {
                    let _ = write!(s, "[bat] {} mV ({}) {}", filtered_mv, tier.label(), cc.label());
                }
                None => {
                    let _ = write!(s, "[bat] {} mV ({})", filtered_mv, tier.label());
                }
            }
            ulog::log(s.as_str());
        }
    }
}

