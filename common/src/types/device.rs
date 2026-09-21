use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HardwareInfo {
    pub make: Option<heapless::String<32>>,
    pub model: Option<heapless::String<32>>,
    pub serial_nr: Option<heapless::String<32>>,
}
