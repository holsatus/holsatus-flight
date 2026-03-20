use common::{
    drivers::imu::{
        icm20948_async::{AccDlp, AccRange, AccUnit, Config, GyrDlp, GyrRange, GyrUnit},
        ImuConfig,
    },
    types::config::{DshotConfig, I2cConfig, UartConfig},
};

pub fn uart0() -> UartConfig {
    UartConfig { baud: 420_000 }
}

pub fn uart1() -> UartConfig {
    UartConfig { baud: 115200 }
}

pub fn i2c1() -> I2cConfig {
    I2cConfig {
        frequency: 400_000,
        sda_pullup: true,
        scl_pullup: true,
    }
}

pub fn dshot() -> DshotConfig {
    DshotConfig::Dshot300
}

pub fn imu() -> ImuConfig {
    ImuConfig::Icm20948(Config {
        acc_range: AccRange::Gs8,
        gyr_range: GyrRange::Dps2000,
        acc_unit: AccUnit::Mpss,
        gyr_unit: GyrUnit::Rps,
        acc_dlp: AccDlp::Hz246,
        gyr_dlp: GyrDlp::Disabled,
        acc_odr: 0,
        gyr_odr: 0,
    })
}

#[cfg(feature = "usb")]
pub fn hw_info() -> common::types::device::HardwareInfo {
    use common::heapless::String;
    use core::str::FromStr;
    common::types::device::HardwareInfo {
        // IMPORTANT: these must not be longer than 32 characters
        make: String::from_str("Peter Krull DIY").ok(),
        model: String::from_str("rp2040-dev-v1").ok(),
        serial_nr: None,
    }
}
