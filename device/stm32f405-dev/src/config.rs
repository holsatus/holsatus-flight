use common::{
    drivers::imu::{
        icm20948_async::{AccDlp, AccRange, AccUnit, Config, GyrDlp, GyrRange, GyrUnit},
        ImuConfig,
    },
    types::config::{DshotConfig, I2cConfig, UartConfig, UartParity, UartStopBits},
};

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
        acc_range: AccRange::Gs4,
        gyr_range: GyrRange::Dps2000,
        acc_unit: AccUnit::Mpss,
        gyr_unit: GyrUnit::Rps,
        acc_dlp: AccDlp::Hz111,
        gyr_dlp: GyrDlp::Hz361,
        acc_odr: 0,
        gyr_odr: 0,
    })
}

pub(crate) fn i2c1() -> I2cConfig {
    I2cConfig {
        // 400kHz is often marginal on dev wiring / weak pullups.
        // Start conservative; bump back up once the bus is electrically solid.
        frequency: 400_000,
        sda_pullup: true,
        scl_pullup: true,
    }
}

pub(crate) fn motor() -> DshotConfig {
    DshotConfig::Dshot300
}

pub(crate) fn usart1() -> UartConfig {
    UartConfig {
        baud: 420_000,
        parity: UartParity::None,
        stop_bits: UartStopBits::One,
        invert_rx: false,
    }
}

pub(crate) fn usart2() -> UartConfig {
    UartConfig {
        baud: 115_200,
        parity: UartParity::None,
        stop_bits: UartStopBits::One,
        invert_rx: false,
    }
}

pub(crate) fn usart3() -> UartConfig {
    UartConfig {
        // SBUS protocol timing/framing.
        baud: 100_000,
        parity: UartParity::Even,
        stop_bits: UartStopBits::Two,
        // On STM32F405 this may require external inversion depending on USART capabilities.
        invert_rx: true,
    }
}

pub(crate) fn usart6() -> UartConfig {
    UartConfig {
        baud: 115_200,
        parity: UartParity::None,
        stop_bits: UartStopBits::One,
        invert_rx: false,
    }
}

#[cfg(feature = "sdmmc")]
pub(crate) fn sdmmc() -> common::types::config::SdmmcConfig {
    common::types::config::SdmmcConfig {
        frequency: 25_000_000,
    }
}

pub(crate) fn flash() -> core::ops::Range<u32> {
    0xC0000..0x100000
}

/// Define the clock configuration for the board.
pub(crate) fn initialize() -> embassy_stm32::Peripherals {
    let mut config = embassy_stm32::Config::default();

    use embassy_stm32::rcc::*;
    config.rcc.hse = Some(Hse{
        freq: embassy_stm32::time::Hertz(12_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV6,
        mul: PllMul::MUL168,
        divp: Some(PllPDiv::DIV2), // 16mhz / 6 * 168 / 2 = 168Mhz.
        divq: Some(PllQDiv::DIV7), // 16mhz / 6 * 168 / 7 = 48Mhz.
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;

    embassy_stm32::init(config)
}
