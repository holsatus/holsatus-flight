//! Board-specific configuration overrides for the MicoAir H743 V2.
//!
//! # Physical motor layout (board silkscreen, arrow = front)
//!
//!        FRONT (arrow)
//!     M2(FL)    M4(FR)
//!       ⟲  \  /  ⟳
//!           \/
//!           /\
//!       ⟳  /  \  ⟲
//!     M1(BL)    M3(BR)
//!        BACK (battery)
//!
//! # Channel-to-pin wiring (fixed by PCB)
//!   Ch1 = PE9  = board M4 = Front-Right
//!   Ch2 = PE11 = board M3 = Back-Right
//!   Ch3 = PE13 = board M2 = Front-Left
//!   Ch4 = PE14 = board M1 = Back-Left
//!
//! # Mixer MOTOR indices (fixed by common/src/airframe)
//!   MOTOR_0 = Back-Right   (CW)
//!   MOTOR_1 = Front-Right  (CCW)
//!   MOTOR_2 = Back-Left    (CCW)
//!   MOTOR_3 = Front-Left   (CW)
//!
//! CHANNEL_MAP remaps MOTOR order to channel order so the mixer's
//! corrections reach the correct physical motors.

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
    // SPI2/3 kernel clock = PER = HSI = 64 MHz
    config.rcc.mux.spi123sel = embassy_stm32::rcc::mux::Saisel::PER;
    // ADC kernel clock = PER = HSI = 64 MHz (required or Adc::new hangs in calibration)
    config.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::PER;
    config
}

/// Maps each DShot channel to the MOTOR index whose speed it should receive.
/// The mixer outputs [MOTOR_0(BR), MOTOR_1(FR), MOTOR_2(BL), MOTOR_3(FL)].
/// DShot channels are [Ch1(FR), Ch2(BR), Ch3(FL), Ch4(BL)].
///   Ch1(FR) <- MOTOR_1  |  Ch2(BR) <- MOTOR_0
///   Ch3(FL) <- MOTOR_3  |  Ch4(BL) <- MOTOR_2
pub const CHANNEL_MAP: [usize; 4] = [1, 0, 3, 2];

/// Reverse flags for this board with corrected CHANNEL_MAP.
/// MOTOR_0(BR), MOTOR_1(FR) need DShot reversal.
/// MOTOR_2(BL), MOTOR_3(FL) spin correctly by default wiring.
pub const MOTOR_REVERSE_FLAGS: Reverse = Reverse::MOTOR_0.union(Reverse::MOTOR_1);

/// BMI088 accelerometer bias [ax, ay, az] in m/s^2.
/// Measured flat on the desk (imu_cal, 2026-04-17). az relative to +g.
/// Re-run imu_cal binary if the FC is remounted or the frame changes.
pub const BMI088_ACC_BIAS: [f32; 3] = [-0.0154, -0.6074, -0.0071];

/// Rotation matrix that maps BMI088 chip-native readings to drone NED body
/// frame (arrow-forward = drone +X, drone right = +Y, drone down = +Z).
/// Empirically verified 2026-04-21 via `level_check` tilt test (see
/// `project_imu_axis_mounting.md` memory).
///
/// Chip mounting on MicoAir H743v2 (right-handed, BMI088 standard labels):
///   chip +X axis points to drone's LEFT (drone -Y)
///   chip +Y axis points to drone's BACKWARD (drone -X)
///   chip +Z axis points UP (drone -Z in NED convention)
///
/// The matrix is column-major (nalgebra convention). Equivalent row-major:
///   drone_ax_forward = -chip_ay
///   drone_ay_right   = -chip_ax
///   drone_az_down    = -chip_az
///
/// Wire into `imu_reader::params::rot` at startup via `override_imu_rot()`
/// in each flight binary. When this is set correctly, `cal_acc.scale` /
/// `cal_gyr.scale` should be `[1.0, 1.0, 1.0]` (identity) -- the rotation
/// handles the axis mapping; scale is just per-axis gain (unity for BMI088).
pub const BMI088_CHIP_TO_DRONE_ROT: [[f32; 3]; 3] = [
    [ 0.0, -1.0,  0.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  0.0, -1.0],
];
