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

pub fn gen_default_cfg() -> BootConfig {
    BootConfig {
        info: HardwareInfo {
            // IMPORTANT: these must not be longer than 32 characters
            make: String::from_str("Peter Krull DIY").ok(),
            model: String::from_str("rp2040-dev-v1").ok(),
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
        uart3: None,
        uart4: None,
        // I2C1 is connected to the main IMU (imu0 -> ICM20948)
        i2c1: Some(I2cConfig {
            goes_to: I2cGoesTo::Accelerometer,
            frequency: 400_000,
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
            gyr_dlp: GyrDlp::Disabled,
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
