use common::types::config::{DshotConfig, UartConfig};

#[cfg(feature = "usb")]
pub(crate) fn hwinfo() -> common::types::device::HardwareInfo {
    use common::heapless::String;
    use core::str::FromStr;
    common::types::device::HardwareInfo {
        // These must not be longer than 32 characters
        make: String::from_str("Micoair").ok(),
        model: String::from_str("H743 V2").ok(),
        serial_nr: None,
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
