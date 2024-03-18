use core::{f32::consts::PI, ops::Range};

use embassy_rp::{
    flash::{Async, Flash},
    peripherals::FLASH,
};
use embassy_time::Duration;
use nalgebra::Vector3;

pub mod storage;
pub mod keyed_item;

pub fn load_config(
    flash: &mut Flash<'static, FLASH, Async, { FLASH_AMT }>,
) -> Option<Configuration> {
    let config: Configuration = unsafe { storage::read(flash, CFG_ADDR_OFFSET) };
    Some(config)
}

pub const DEFAULT_CONFIG: Configuration = RP2040_DEV_CONFIG;

use icm20948_async::*;
use sequential_storage::map::StorageItem;
use crate::{airframe::MotorMixing, common::types::MavStreamableFrequencies, filters::pid_controller::PidConfig};

use crate::{
    common::rotation_matrices,
    transmitter::TransmitterMap,
};
pub const RP2040_DEV_CONFIG: Configuration = Configuration {
    key: CFG_KEY,
    imu_cfg: [Some(ImuConfig {
            acc_cal: Some(Calibration {
                scale: Vector3::new(0.9941048, 0.9951916, 0.987686),
                offset: Vector3::new(-0.08339453, -0.21677876, 0.27320194)
            }),
        gyr_cal: Some(Calibration {
            scale: Vector3::new(1.0, 1.0, 1.0),
            offset: Vector3::new(-0.01429255, 0.020241564, 0.0021124864)
        }),
        imu_ext: Some(Extrinsics {
            rotation: rotation_matrices::Rotation::RotX180,
            translation: VEC_ZEROS,
        }),
        imu_type: Some(ImuTypeConf::Icm20948(icm20948_async::Icm20948Config {
            acc_range: AccRange::Gs8,
            gyr_range: GyrRange::Dps2000,
            acc_unit: AccUnit::Mpss,
            gyr_unit: GyrUnit::Rps,
            acc_dlp: AccDlp::Hz111,
            gyr_dlp: GyrDlp::Hz361,
            acc_odr: 0,
            gyr_odr: 0,
        })),
    }), None],
    imu_freq: 1000,
    mag_cfg: [Some(MagConfig {
        mag_cal: None,
        mag_ext: None,
    }), None],
    mag_freq: 100,
    attpids: AttitudePids {
        roll_inner: PidConfig {
            kp: 30.,
            ki: 1.0,
            kd: 0.01,
            ideal: true,
            wrapping: None,
            output_limit: None,
            anti_windup: None,
            lp_filter: Some(0.01),
        },
        roll_outer: PidConfig {
            kp: 15.,
            ki: 0.1,
            kd: 0.0,
            ideal: true,
            wrapping: Some((-PI, PI)),
            output_limit: None,
            anti_windup: None,
            lp_filter: None,
        },
        pitch_inner: PidConfig {
            kp: 35.,
            ki: 1.0,
            kd: 0.01,
            ideal: true,
            wrapping: None,
            output_limit: None,
            anti_windup: None,
            lp_filter: Some(0.01),
        },
        pitch_outer: PidConfig {
            kp: 15.,
            ki: 0.1,
            kd: 0.0,
            ideal: true,
            wrapping: Some((-PI, PI)),
            output_limit: None,
            anti_windup: None,
            lp_filter: None,
        },
        yaw_inner: PidConfig {
            kp: 60.,
            ki: 1.0,
            kd: 0.01,
            ideal: true,
            wrapping: None,
            output_limit: None,
            anti_windup: None,
            lp_filter: Some(0.01),
        },
        yaw_outer: PidConfig {
            kp: 8.,
            ki: 1e-3,
            kd: 0.0,
            ideal: true,
            wrapping: Some((-PI, PI)),
            output_limit: None,
            anti_windup: None,
            lp_filter: None,
        },
    },
    dshot_speed: DshotSpeed::Dshot300,
    motor_dir: [false, false, false, true],
    mav_freq: MavStreamableFrequencies {
        heartbeat: Some(Duration::from_hz(1)),
        system_time: None,
        attitude: Some(Duration::from_hz(20)),
        gps_raw_int: None,
        scaled_imu: None,
        rc_channels: None,
    },
    mixing_matrix: MotorMixing::QuadX,
    radio_timeout: Duration::from_millis(200),
    dshot_timeout: Duration::from_millis(100),
    tx_map: crate::transmitter::tx_12_profiles::TX12_8CH_DEFAULT_MAP,
};


pub const FLASH_AMT: usize = 2 * 1024 * 1024;
pub const CFG_ADDR_OFFSET: u32 = 0x100000;


// Headers are used to check that loaded configuration is not just garbage data
pub const CFG_KEY: u8 = 127;

// Start and end of the configuration and queue flash ranges
const MAP_RANGE_START: u32 = CFG_ADDR_OFFSET;
const MAP_RANGE_END: u32 = MAP_RANGE_START + 1024 * 256;
const QUEUE_RANGE_START: u32 = MAP_RANGE_END;
const QUEUE_RANGE_END: u32 = QUEUE_RANGE_START + 1024 * 512;

pub const CFG_MAP_RANGE: Range<u32> = MAP_RANGE_START..MAP_RANGE_END;
pub const CFG_QUEUE_RANGE: Range<u32> = QUEUE_RANGE_START..QUEUE_RANGE_END;

#[repr(C)]
pub struct DataStore<T, K> {
    key: K,
    data: T
}
macro_rules! make_data_store {
    ($datatype:ty, $key:ty, $make_key:expr) => {
                
        impl StorageItem for DataStore<$datatype,$key> {
            type Key = $key;

            type Error = ();

            fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {

                // Ensure buffer is large enough to contain the data
                if buffer.len() < core::mem::size_of::<Self>() {
                    return Err(());
                }

                // Get the bytes of the data as a slice referencing the data
                let data_bytes =
                    unsafe { core::slice::from_raw_parts((self as *const Self) as *const u8, core::mem::size_of::<Self>()) };

                // Copy the data bytes into the buffer
                buffer[..core::mem::size_of::<Self>()].copy_from_slice(data_bytes);

                // Return the size of the data
                Ok(core::mem::size_of::<Self>())
            }

            fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error>
            where
                Self: Sized,
            {   
                // Ensure buffer is large enough to contain the data
                if buffer.len() < core::mem::size_of::<Self>() {
                    return Err(());
                }

                // Relevant slice of original buffer
                let smaller_buffer = &buffer[..core::mem::size_of::<Self>()];

                // Create a new buffer to store the data
                let mut datastore = [0u8; core::mem::size_of::<Self>()];

                // Copy the data from the smaller buffer into the datastore
                for (i, byte) in smaller_buffer.iter().enumerate() {
                    datastore[i] = *byte;
                }

                // "Deserialize" the data via transmutation
                let data: Self = unsafe { core::mem::transmute(datastore) };

                // Return the data
                Ok(data)
            }

            fn key(&self) -> Self::Key {
                self.key
            }

            fn deserialize_key_only(buffer: &[u8]) -> Result<Self::Key, Self::Error>
            where
                Self: Sized,
            {
                ($make_key)(buffer)
            }
        }
    };
}

make_data_store!(Configuration, u16, |buffer: &[u8]|Ok(u16::from_be_bytes([buffer[0], buffer[1]])));

#[derive(Debug, Clone, Copy)]
pub struct Configuration {
    pub key: u8,
    pub imu_freq: u16,
    pub imu_cfg: [Option<ImuConfig>; crate::N_IMU],
    pub mag_freq: u16,
    pub mag_cfg: [Option<MagConfig>; crate::N_MAG],
    pub attpids: AttitudePids,
    pub dshot_speed: DshotSpeed,
    pub motor_dir: [bool; 4],
    pub mav_freq: MavStreamableFrequencies,
    pub mixing_matrix: MotorMixing,
    pub radio_timeout: Duration,
    pub dshot_timeout: Duration,
    pub tx_map: TransmitterMap,
}

pub type Cfg = crate::config::keyed_item::KeyedItem<Configuration, u8>;

#[derive(Debug, Clone, Copy)]
pub struct AttitudePids {
    pub roll_inner: PidConfig<f32>,
    pub roll_outer: PidConfig<f32>,
    pub pitch_inner: PidConfig<f32>,
    pub pitch_outer: PidConfig<f32>,
    pub yaw_inner: PidConfig<f32>,
    pub yaw_outer: PidConfig<f32>,
}

pub enum Type {
    F64(f64),
    F32(f32),
    Isize(isize),
    I64(i64),
    I32(i32),
    I16(i16),
    I8(i8),
    Usize(usize),
    U64(u64),
    U32(u32),
    U16(u16),
    U8(u8),
    Bool(bool),
    Unknown,
    None,
}

pub trait ParamLookup {
    /// Look up a system parameter by name, returns either a float or an integer if the parameter is found.
    /// That is, the result will only ever be `(Some(f32), None)`, `(None, Some(i32)` or `(None, None)`.
    fn get(&self, name: core::str::SplitInclusive<'_, char>) -> Type;
    fn set(&self, name: core::str::SplitInclusive<'_, char>, val: Type) -> Type;
}

impl ParamLookup for Configuration {
    fn get(&self, mut splits: core::str::SplitInclusive<'_, char>) -> Type {
        match splits.next() {
            Some("attpid_") => self.attpids.get(splits),
            _ => Type::Unknown,
        }
    }

    fn set(&self, mut _splits: core::str::SplitInclusive<'_, char>, _val: Type) -> Type {
        todo!()
    }
}

impl ParamLookup for AttitudePids {

    fn get(&self, mut splits: core::str::SplitInclusive<'_, char>) -> Type {

        enum Axis {
            Roll,
            Pitch,
            Yaw,
        }

        enum Layer {
            Inner,
            Outer,
        }

        enum Param {
            Kp,
            Ki,
            Kd,
            OutLimMax,
            OutLimMin,
            LpTau,
        }

        let axis = match splits.next() {
            Some("roll_") => Axis::Roll,
            Some("pitch_") => Axis::Pitch,
            Some("yaw_") => Axis::Yaw,
            _ => return Type::Unknown,
        };

        let layer = match splits.next() {
            Some("inner_") => Layer::Inner,
            Some("outer_") => Layer::Outer,
            _ => return Type::Unknown,
        };

        let param = match splits.next() {
            Some("kp") => Param::Kp,
            Some("ki") => Param::Ki,
            Some("kd") => Param::Kd,
            Some("outlimmax") => Param::OutLimMax,
            Some("outlimmin") => Param::OutLimMin,
            Some("lptau") => Param::LpTau,
            _ => return Type::Unknown,
        };

        match (axis, layer, param) {
            (Axis::Roll, Layer::Inner, Param::Kp) => Type::F32(self.roll_inner.kp),
            (Axis::Roll, Layer::Inner, Param::Ki) => Type::F32(self.roll_inner.ki),
            (Axis::Roll, Layer::Inner, Param::Kd) => Type::F32(self.roll_inner.kd),
            (Axis::Roll, Layer::Inner, Param::OutLimMax) => self.roll_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Roll, Layer::Inner, Param::OutLimMin) => self.roll_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Roll, Layer::Inner, Param::LpTau) => self.roll_inner.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
            (Axis::Roll, Layer::Outer, Param::Kp) => Type::F32(self.roll_outer.kp),
            (Axis::Roll, Layer::Outer, Param::Ki) => Type::F32(self.roll_outer.ki),
            (Axis::Roll, Layer::Outer, Param::Kd) => Type::F32(self.roll_outer.kd),
            (Axis::Roll, Layer::Outer, Param::OutLimMax) => self.roll_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Roll, Layer::Outer, Param::OutLimMin) => self.roll_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Roll, Layer::Outer, Param::LpTau) => self.roll_outer.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
            (Axis::Pitch, Layer::Inner, Param::Kp) => Type::F32(self.pitch_inner.kp),
            (Axis::Pitch, Layer::Inner, Param::Ki) => Type::F32(self.pitch_inner.ki),
            (Axis::Pitch, Layer::Inner, Param::Kd) => Type::F32(self.pitch_inner.kd),
            (Axis::Pitch, Layer::Inner, Param::OutLimMax) => self.pitch_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Pitch, Layer::Inner, Param::OutLimMin) => self.pitch_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Pitch, Layer::Inner, Param::LpTau) => self.pitch_inner.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
            (Axis::Pitch, Layer::Outer, Param::Kp) => Type::F32(self.pitch_outer.kp),
            (Axis::Pitch, Layer::Outer, Param::Ki) => Type::F32(self.pitch_outer.ki),
            (Axis::Pitch, Layer::Outer, Param::Kd) => Type::F32(self.pitch_outer.kd),
            (Axis::Pitch, Layer::Outer, Param::OutLimMax) => self.pitch_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Pitch, Layer::Outer, Param::OutLimMin) => self.pitch_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Pitch, Layer::Outer, Param::LpTau) => self.pitch_outer.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
            (Axis::Yaw, Layer::Inner, Param::Kp) => Type::F32(self.yaw_inner.kp),
            (Axis::Yaw, Layer::Inner, Param::Ki) => Type::F32(self.yaw_inner.ki),
            (Axis::Yaw, Layer::Inner, Param::Kd) => Type::F32(self.yaw_inner.kd),
            (Axis::Yaw, Layer::Inner, Param::OutLimMax) => self.yaw_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Yaw, Layer::Inner, Param::OutLimMin) => self.yaw_inner.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Yaw, Layer::Inner, Param::LpTau) => self.yaw_inner.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
            (Axis::Yaw, Layer::Outer, Param::Kp) => Type::F32(self.yaw_outer.kp),
            (Axis::Yaw, Layer::Outer, Param::Ki) => Type::F32(self.yaw_outer.ki),
            (Axis::Yaw, Layer::Outer, Param::Kd) => Type::F32(self.yaw_outer.kd),
            (Axis::Yaw, Layer::Outer, Param::OutLimMax) => self.yaw_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.max)),
            (Axis::Yaw, Layer::Outer, Param::OutLimMin) => self.yaw_outer.output_limit.map_or(Type::None, |lim| Type::F32(lim.min)),
            (Axis::Yaw, Layer::Outer, Param::LpTau) => self.yaw_outer.lp_filter.map_or(Type::None, |tau| Type::F32(tau)),
        }
    }
    
    fn set(&self, splits: core::str::SplitInclusive<'_, char>, _val: Type) -> Type {
        match splits {
            _ => todo!()
        }
    }

}


#[derive(Debug, Clone, Copy)]
pub struct ImuConfig {
    ///Accelerometer calibration
    pub acc_cal: Option<Calibration>,
    /// Gyroscope calibration
    pub gyr_cal: Option<Calibration>,
    /// IMU rotation and translation
    pub imu_ext: Option<Extrinsics>,
    /// IMU type and configuration
    pub imu_type: Option<ImuTypeConf>,
}

#[derive(Debug, Clone, Copy)]
pub enum ImuTypeConf {
    Icm20948(Icm20948Config),
}

#[derive(Debug, Clone, Copy)]
pub enum ImuType {
    Icm20948,
}

impl Default for ImuConfig {
    fn default() -> Self {
        ImuConfig {
            acc_cal: None,
            gyr_cal: None,
            imu_ext: None,
            imu_type: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MagConfig {
    ///Magnetometer calibration
    pub mag_cal: Option<Calibration>,
    /// Magnetometer rotation (XYZ)
    pub mag_ext: Option<Extrinsics>,
}

impl Default for MagConfig {
    fn default() -> Self {
        Self {
            mag_cal: None,
            mag_ext: Default::default(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    pub scale: Vector3<f32>,
    pub offset: Vector3<f32>,
}

impl Calibration {
    pub fn apply(&self, sample: &mut Vector3<f32>) {
        sample.zip_zip_apply(&self.offset, &self.scale, |x, o, s| *x = (*x - o) * s);
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            scale: VEC_ONES,
            offset: VEC_ZEROS,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Extrinsics {
    /// The rotation matrix (function) to apply to the
    /// sensor data to align it with the NED body frame.
    pub rotation: rotation_matrices::Rotation,

    /// The translation vector which describes the sensor
    /// position relative to the body frame.
    pub translation: Vector3<f32>,
}

impl Default for Extrinsics {
    fn default() -> Self {
        Self {
            rotation: rotation_matrices::Rotation::Identity,
            translation: VEC_ZEROS,
        }
    }
}

impl Extrinsics {
    pub fn apply(&self, sample: &mut Vector3<f32>) {
        *sample = self.rotation*(*sample - self.translation)
    }
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum DshotSpeed {
    Dshot150,
    Dshot300,
    Dshot600,
    Dshot1200,
}

impl Default for DshotSpeed {
    fn default() -> Self {
        DshotSpeed::Dshot150
    }
}

// TODO calculate this from configured system clock
#[cfg(feature = "overclock")]
impl DshotSpeed {
    pub const fn clk_div(&self) -> (u16, u8) {
        match self {
            DshotSpeed::Dshot150 => (208, 0),
            DshotSpeed::Dshot300 => (104, 0),
            DshotSpeed::Dshot600 => (52, 0),
            DshotSpeed::Dshot1200 => (26, 0),
        }
    }
}

// TODO calculate this from configured system clock
#[cfg(not(feature = "overclock"))]
impl DshotSpeed {
    pub const fn clk_div(&self) -> (u16, u8) {
        match self {
            DshotSpeed::Dshot150 => (104, 0),
            DshotSpeed::Dshot300 => (52, 0),
            DshotSpeed::Dshot600 => (26, 0),
            DshotSpeed::Dshot1200 => (13, 0),
        }
    }
}

const VEC_ZEROS: Vector3<f32> = Vector3::new(0., 0., 0.);
const VEC_ONES: Vector3<f32> = Vector3::new(1., 1., 1.);
