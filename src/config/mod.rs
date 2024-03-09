use core::f32::consts::PI;

use embassy_rp::{
    flash::{Async, Flash},
    peripherals::FLASH,
};
use embassy_time::Duration;
use nalgebra::Vector3;

pub mod storage;

pub fn load_config(
    flash: &mut Flash<'static, FLASH, Async, { FLASH_AMT }>,
) -> Option<Configuration> {
    let config: Configuration = unsafe { storage::read(flash, CFG_ADDR_OFFSET) };
    if config.header == DEFAULT_CONFIG.header && config.footer == DEFAULT_CONFIG.footer {
        Some(config)
    } else {
        None
    }
}

pub const DEFAULT_CONFIG: Configuration = RP2040_DEV_CONFIG;

use icm20948_async::*;
use crate::{airframe::MotorMixing, filters::pid_controller::PidConfig};

use crate::{
    common::rotation_matrices, mavlink::supported_msg::MavStreamableFrequencies,
    transmitter::TransmitterMap,
};
pub const RP2040_DEV_CONFIG: Configuration = Configuration {
    header: CFG_HEADER,
    imu_cfg: [Some(ImuConfig {
            acc_cal: Some(Calibration {
                scale: Vector3::new(0.99470156, 0.99557877, 0.987214),
                offset: Vector3::new(0.09380102, -0.24849701, -0.250834)
            }),
        gyr_cal: Some(Calibration {
            scale: Vector3::new(1.0, 1.0, 1.0),
            offset: Vector3::new(0.013425202, 0.02018836, -0.0020406505)
        }),
        imu_ext: Some(Extrinsics {
            rotation: rotation_matrices::Rotation::RotY180,
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
    pids: AttitudePids {
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
    motor_gov_timeout: Duration::from_millis(100),
    tx_map: crate::transmitter::tx_12_profiles::TX12_DEFAULT_MAP,
    footer: CFG_FOOTER,
};


pub const FLASH_AMT: usize = 2 * 1024 * 1024;
pub const CFG_ADDR_OFFSET: u32 = 0x100000;

// Headers are used to check that loaded configuration is not just garbage data
pub const CFG_HEADER: u64 = 252622182913624215;
pub const CFG_FOOTER: u64 = 145389224228254269;

#[derive(Debug, Clone, Copy)]
pub struct Configuration {
    pub header: u64,
    pub imu_freq: u16,
    pub imu_cfg: [Option<ImuConfig>; crate::N_IMU],
    pub mag_freq: u16,
    pub mag_cfg: [Option<MagConfig>; crate::N_MAG],
    pub pids: AttitudePids,
    pub dshot_speed: DshotSpeed,
    pub motor_dir: [bool; 4],
    pub mav_freq: MavStreamableFrequencies,
    pub mixing_matrix: MotorMixing,
    pub radio_timeout: Duration,
    pub motor_gov_timeout: Duration,
    pub tx_map: TransmitterMap,
    pub footer: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct AttitudePids {
    pub roll_inner: PidConfig<f32>,
    pub roll_outer: PidConfig<f32>,
    pub pitch_inner: PidConfig<f32>,
    pub pitch_outer: PidConfig<f32>,
    pub yaw_inner: PidConfig<f32>,
    pub yaw_outer: PidConfig<f32>,
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

#[cfg(feature = "overclock")]
impl DshotSpeed {
    pub fn clk_div(&self) -> (u16, u8) {
        match self {
            DshotSpeed::Dshot150 => (208, 0),
            DshotSpeed::Dshot300 => (104, 0),
            DshotSpeed::Dshot600 => (52, 0),
            DshotSpeed::Dshot1200 => (26, 0),
        }
    }
}

#[cfg(not(feature = "overclock"))]
impl DshotSpeed {
    pub fn clk_div(&self) -> (u16, u8) {
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
