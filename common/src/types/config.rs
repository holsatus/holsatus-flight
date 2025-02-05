use sequential_storage::map::{SerializationError, Value};
use serde::{Deserialize, Serialize};

use crate::{
    airframe::QuadRotorMixing, calibration::sens3d::Calib3DType, drivers::imu::ImuConfig, filters::rate_pid::RatePidCfg3D, rc_mapping::RcBindings, utils::rot_matrix::Rotation, NUM_IMU, NUM_MAG
};

use super::device::HardwareInfo;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct UartConfig {
    pub goes_to: UartGoesTo,
    pub baud: u32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum UartGoesTo {
    RcControl,
    MAVLink,
    Gnss,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct I2cConfig {
    pub goes_to: I2cGoesTo,
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ImuIntrinsics {
    pub imu_rot: Rotation,
    pub acc_cal: Calib3DType,
    pub gyr_cal: Calib3DType,
}


#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MagIntrinsics {
    pub mag_rot: Rotation,
    pub mag_cal: Calib3DType,
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub motor_mix: QuadRotorMixing,
    pub motor_dirs: [bool; 4],
    pub motor_tau_ms: u16,
    pub mass: f32,
}

impl Default for Vehicle {
    fn default() -> Self {
        Vehicle {
            motor_mix: QuadRotorMixing::QuadX,
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

pub trait Keyed<'a>: Serialize + Deserialize<'a> {
    const KEY: u8;
}

macro_rules! impl_keyed {
    ($($key:literal => $type:ty $(,)?)*  ) => {
        $(
            impl <'a> Keyed<'a> for $type {
                const KEY: u8 = $key;
            }
        )*
    };
}

impl_keyed! {
    0x00 => BootConfig
    0x11 => ImuIntrinsics
    0x12 => MagIntrinsics
    0x20 => RcBindings
    0x30 => Vehicle
    0x40 => RatePidCfg3D
}

#[cfg(feature = "mavlink")]
impl_keyed! {
    0xF0 => crate::mavlink::MavStreamCfg
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Wrap<T>(pub T);

impl<'a, T: Serialize + Deserialize<'a>> Value<'a> for Wrap<T> {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer).map(|slice| slice.len()).map_err(|e| match e {
            postcard::Error::SerializeBufferFull => SerializationError::BufferTooSmall,
            _ => SerializationError::InvalidFormat,
        })
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized {
        postcard::from_bytes(buffer).map_err(|e| match e {
            postcard::Error::SerializeBufferFull => SerializationError::BufferTooSmall,
            _ => SerializationError::InvalidFormat,
        })
    }
}
