use serde::{Deserialize, Serialize};

use crate::{
    calibration::sens3d::Calib3D, drivers::imu::ImuConfig,
    utils::rot_matrix::Rotation,
};

use super::device::HardwareInfo;

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

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum I2cGoesTo {
    Barometer,
    Magnetometer,
    Accelerometer,
    Gyroscope,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdmmcConfig {
    pub frequency: u32,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BootConfig {
    pub info: HardwareInfo,
    pub uart1: Option<UartConfig>,
    pub uart2: Option<UartConfig>,
    pub uart3: Option<UartConfig>,
    pub uart4: Option<UartConfig>,
    pub i2c1: Option<I2cConfig>,
    pub i2c2: Option<I2cConfig>,
    pub imu0: Option<ImuConfig>,
    pub sdmmc: Option<SdmmcConfig>,
    pub motors: Option<DshotConfig>,
}

#[derive(mav_param::Tree, Debug, Clone)]
pub struct ImuIntrinsics {
    pub imu_rot: Rotation,
    pub acc_cal: Calib3D,
    pub gyr_cal: Calib3D,
}

impl ImuIntrinsics {
    pub const fn const_default() -> Self {
        ImuIntrinsics {
            imu_rot: Rotation::const_default(),
            acc_cal: Calib3D::const_default(),
            gyr_cal: Calib3D::const_default(),
        }
    }
}

#[derive(mav_param::Tree, Debug, Clone)]
pub struct MagIntrinsics {
    pub mag_rot: Rotation,
    pub mag_cal: Calib3D,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub motor_dirs: [bool; 4],
    pub motor_tau_ms: u16,
    pub mass: f32,
}

impl Default for Vehicle {
    fn default() -> Self {
        Vehicle {
            motor_dirs: [true, false, false, true],
            motor_tau_ms: 50,
            mass: 0.65,
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum DshotConfig {
    Dshot150 = 150,
    Dshot300 = 300,
    Dshot600 = 600,
    Dshot1200 = 1200,
}
