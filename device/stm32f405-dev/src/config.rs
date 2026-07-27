use common::{
    drivers::imu::icm20948::{
        AccDlp, AccRange, AccUnit, Config as IcmConfig, GyrDlp, GyrRange, GyrUnit,
    },
    types::config::{DshotConfig, I2cConfig, UartConfig},
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

pub(crate) fn imu() -> IcmConfig {
    IcmConfig {
        acc_range: AccRange::Gs8,
        gyr_range: GyrRange::Dps2000,
        acc_unit: AccUnit::Mpss,
        gyr_unit: GyrUnit::Rps,
        acc_dlp: AccDlp::Hz111,
        gyr_dlp: GyrDlp::Hz361,
        acc_odr: 0,
        gyr_odr: 0,
    }
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

pub(crate) fn usart2() -> UartConfig {
    UartConfig { baud: 115_200 }
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
    0xC0000..0x100000
}
