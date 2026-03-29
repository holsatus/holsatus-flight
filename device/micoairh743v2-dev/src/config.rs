//! Board-specific configuration overrides for the MicoAir H743.
//!
//! Motor channel mapping (TIM1, DshotDriver::new argument order):
//!   Ch1 -> M4 (PE9)   -> MOTOR_0 in motor_governor indexing
//!   Ch2 -> M3 (PE11)  -> MOTOR_1
//!   Ch3 -> M2 (PE13)  -> MOTOR_2
//!   Ch4 -> M1 (PE14)  -> MOTOR_3
//!
//! Physical observation (motor_dir_test, all-four-reversed config):
//!   MOTOR_0 (back-right):  CW  -- correct, keep reversed
//!   MOTOR_1 (front-right): CCW -- correct, keep reversed
//!   MOTOR_2 (back-left):   CW  -- wrong (needs CCW), remove from reverse flags
//!   MOTOR_3 (front-left):  CW  -- correct, keep reversed
//! Result: MOTOR_0 | MOTOR_1 | MOTOR_3 reversed; MOTOR_2 not reversed.

use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale};
use embassy_stm32::rcc::HSIPrescaler;
pub use common::tasks::motor_governor::params::Reverse;

/// Return an embassy_stm32::Config with PLL1 configured for 400 MHz SYSCLK.
///
/// Clock tree (VOS1, LDO supply):
///   HSI (64 MHz) -> PLL1 prediv /4 -> 16 MHz -> x50 -> 800 MHz VCO
///   PLL1_P = 800 / 2 = 400 MHz  (SYSCLK, CPU)
///   PLL1_Q = 800 / 4 = 200 MHz  (available for peripherals)
///   HCLK   = 400 / 2 = 200 MHz  (AHB, AXI SRAM)
///   APB1/2/3/4 = 200 / 2 = 100 MHz
///
/// HSI is kept running as the PER clock so SPI2 kernel clock (set via
/// spi123sel = PER) stays at 64 MHz, matching the SPI divisor already chosen.
///
/// VOS1 limits (embassy stm32h7): CPU <= 400 MHz, HCLK <= 200 MHz,
/// APB <= 100 MHz -- this config sits exactly at those limits.
pub fn embassy_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(HSIPrescaler::DIV1); // HSI = 64 MHz; also used as PER clock
    config.rcc.csi = true;                     // needed for some H7 peripheral muxes
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4, // 64 / 4 = 16 MHz
        mul: PllMul::MUL50,      // 16 * 50 = 800 MHz VCO
        fracn: None,
        divp: Some(PllDiv::DIV2), // 800 / 2 = 400 MHz SYSCLK
        divq: Some(PllDiv::DIV4), // 800 / 4 = 200 MHz
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;            // SYSCLK = 400 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV2;    // HCLK   = 200 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2;   // APB1   = 100 MHz (I2C2, USART1, TIM2)
    config.rcc.apb2_pre = APBPrescaler::DIV2;   // APB2   = 100 MHz (TIM1, SPI1)
    config.rcc.apb3_pre = APBPrescaler::DIV2;   // APB3   = 100 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2;   // APB4   = 100 MHz
    config.rcc.voltage_scale = VoltageScale::Scale1; // VOS1
    // SPI2 kernel clock = PER = HSI = 64 MHz (unchanged from previous config)
    config.rcc.mux.spi123sel = embassy_stm32::rcc::mux::Saisel::PER;
    config
}

/// Reverse flags for this board. MOTOR_0, MOTOR_1, MOTOR_3 are reversed.
pub const MOTOR_REVERSE_FLAGS: Reverse = Reverse::MOTOR_0.union(Reverse::MOTOR_1).union(Reverse::MOTOR_3);

// ------------------------------------------------------------------
// Compile-time guards -- these fire on every build, not just cargo test.
// ------------------------------------------------------------------

// Verify exactly MOTOR_0 | MOTOR_1 | MOTOR_3 are reversed.
const EXPECTED_BITS: u16 = Reverse::MOTOR_0.bits() | Reverse::MOTOR_1.bits() | Reverse::MOTOR_3.bits();
const _: () = assert!(
    MOTOR_REVERSE_FLAGS.bits() == EXPECTED_BITS,
    "MOTOR_REVERSE_FLAGS must be MOTOR_0 | MOTOR_1 | MOTOR_3 for this board"
);
