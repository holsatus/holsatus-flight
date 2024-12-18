use core::str::FromStr;

use common::{
    drivers::imu::{
        icm20948_async::{AccDlp, AccRange, AccUnit, GyrDlp, GyrRange, GyrUnit, Icm20948Config},
        ImuConfig,
    },
    heapless::String,
    types::{
        config::{
            BootConfig, DshotConfig, I2cConfig, I2cGoesTo, SdmmcConfig, UartConfig, UartGoesTo,
        },
        device::HardwareInfo,
    },
};

pub(crate) const BLACKBOX_BUF: usize = 512 * 2;

pub fn gen_default_cfg() -> BootConfig {
    BootConfig {
        info: HardwareInfo {
            // IMPORTANT: these must not be longer than 32 characters
            make: String::from_str("Peter Krull DIY").ok(),
            model: String::from_str("stm32f405-dev-v1").ok(),
            serial_nr: None,
        },
        // UART1 (hw UART1) is connected to the RC control with ELRS
        uart1: Some(UartConfig {
            goes_to: UartGoesTo::RcControl,
            baud: 420_000,
        }),
        // UART2 (hw UART6) is connected to the GPS module
        uart2: Some(UartConfig {
            goes_to: UartGoesTo::Gnss,
            baud: 115_200,
        }),
        // UART3 (hw UART3) is connected to the MAVLink server
        uart3: Some(UartConfig {
            goes_to: UartGoesTo::MAVLink,
            baud: 115_200,
        }),
        uart4: None,
        // I2C1 is connected to the main IMU (imu0 -> ICM20948)
        i2c1: Some(I2cConfig {
            goes_to: I2cGoesTo::Accelerometer,
            frequency: 1_000_000,
            sda_pullup: true,
            scl_pullup: true,
        }),
        i2c2: None,
        // Configuration for the ICM20948 IMU
        imu0: Some(ImuConfig::Icm20948(Icm20948Config {
            acc_range: AccRange::Gs8,
            gyr_range: GyrRange::Dps2000,
            acc_unit: AccUnit::Mpss,
            gyr_unit: GyrUnit::Rps,
            acc_dlp: AccDlp::Hz246,
            gyr_dlp: GyrDlp::Hz361,
            acc_odr: 0,
            gyr_odr: 0,
            int: None,
        })),
        sdmmc: Some(SdmmcConfig {
            frequency: 25_000_000,
        }),
        // Motor protocol is DShot300
        motors: Some(DshotConfig::Dshot300),
    }
}


/// Define the clock configuration for the board.
pub(crate) fn initialize() -> embassy_stm32::Peripherals {
    let mut config = embassy_stm32::Config::default();
    
    use embassy_stm32::rcc::*;
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32::time::Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL168,
        divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
        divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    
    embassy_stm32::init(config)
}
