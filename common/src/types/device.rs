use serde::{Deserialize, Serialize};

// #[derive(Debug, Clone, Serialize, Deserialize)]
// pub struct DeviceInformation {
//     pub make: &'static str,
//     pub model: &'static str,
//     pub imu: [Option<&'static str>; 4],
//     pub interrupt_executors: u8,
//     pub has_sd_card: bool,
// }

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HardwareInfo {
    pub make: Option<heapless::String<32>>,
    pub model: Option<heapless::String<32>>,
    pub serial_nr: Option<heapless::String<32>>,
}
