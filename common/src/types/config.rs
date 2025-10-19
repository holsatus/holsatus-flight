use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct UartConfig {
    pub baud: u32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct I2cConfig {
    pub frequency: u32,
    pub sda_pullup: bool,
    pub scl_pullup: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdmmcConfig {
    pub frequency: u32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum DshotConfig {
    Dshot150 = 150,
    Dshot300 = 300,
    Dshot600 = 600,
    Dshot1200 = 1200,
}
