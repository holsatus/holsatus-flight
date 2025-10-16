use common::{
    drivers::imu::{
        icm20948_async::{AccDlp, AccRange, AccUnit, Config, GyrDlp, GyrRange, GyrUnit},
        ImuConfig,
    },
    types::config::{DshotConfig, I2cConfig, UartConfig},
};

#[cfg(feature = "sdmmc")]
pub(crate) const BLACKBOX_BUF: usize = 512 * 2;

#[cfg(feature = "usb")]
pub(crate) fn hwinfo() -> common::types::device::HardwareInfo {
    use common::heapless::String;
    use core::str::FromStr;
    common::types::device::HardwareInfo {
        // These must not be longer than 32 characters
        make: String::from_str("Peter Krull DIY").ok(),
        model: String::from_str("stm32f405-dev-v1").ok(),
        serial_nr: None,
    }
}

pub(crate) fn imu() -> ImuConfig {
    ImuConfig::Icm20948(Config {
        acc_range: AccRange::Gs8,
        gyr_range: GyrRange::Dps2000,
        acc_unit: AccUnit::Mpss,
        gyr_unit: GyrUnit::Rps,
        acc_dlp: AccDlp::Hz246,
        gyr_dlp: GyrDlp::Hz361,
        acc_odr: 0,
        gyr_odr: 0,
    })
}

pub(crate) fn i2c1() -> I2cConfig {
    I2cConfig {
        frequency: 1_000_000,
        sda_pullup: true,
        scl_pullup: true,
    }
}

pub(crate) fn motor() -> DshotConfig {
    DshotConfig::Dshot300
}

pub(crate) fn usart1() -> UartConfig {
    UartConfig { baud: 420_000 }
}

pub(crate) fn usart3() -> UartConfig {
    UartConfig { baud: 115_200 }
}

pub(crate) fn usart6() -> UartConfig {
    UartConfig { baud: 115_200 }
}

#[cfg(feature = "sdmmc")]
pub(crate) fn sdmmc() -> common::types::config::SdmmcConfig {
    common::types::config::SdmmcConfig {
        frequency: 25_000_000,
    }
}

pub(crate) fn flash() -> core::ops::Range<u32> {
    0x80000..0x100000
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
