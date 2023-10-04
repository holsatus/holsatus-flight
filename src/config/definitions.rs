use embassy_time::Duration;
use nalgebra::{Vector3, Matrix3};

pub static SBUS_PARSE_TIMEOUT: Duration = Duration::from_millis(500);
pub static MOTOR_GOV_TIMEOUT: Duration = Duration::from_millis(100);
pub static ATTITUDE_LOOP_TIME_DUR: Duration = Duration::from_hz(400);
pub static ATTITUDE_LOOP_TIME_SECS: f32 = ATTITUDE_LOOP_TIME_DUR.as_micros() as f32 / 1e6;

pub const DEFAULT_CONFIG: Configuration = RP2040_DEV_CONFIG;

pub const RP2040_DEV_CONFIG: Configuration = Configuration{
    header: CFG_HEADER,
    imu0: Some(ImuFeatures {
        acc_cal: None,
        gyr_cal: None,
        imu_ext: Extrinsics {
            rotation: MAT_NEGATE_YZ,
            translation: VEC_ZEROS
        },
    }),
    imu1: None,
    imu2: None,
    imu3: None,
    mag0: Some(MagFeatures {
        mag_cal: None,
        mag_ext: Extrinsics {
            rotation: MAT_IDENTITY,
            translation: VEC_ZEROS
        },
    }),
    mag1: None,
    mag2: None,
    mag3: None,
    dshot_speed: DshotSpeed::Dshot300,
    motor_dir: [false, false, false, true],
    attitude_loop_time: Duration::from_hz(400),
    mav_freq: MavlinkMsgFrequencies { 
        heartbeat: Some(Duration::from_hz(1)),
    },
    footer: CFG_FOOTER,
};

pub static TAKEOFF_ESTIMATOR_THRUST: u16 = 700;

pub const FLASH_AMT: usize = 2 * 1024 * 1024;
pub const CFG_ADDR_OFFSET: u32 = 0x100000;

pub const CFG_HEADER: u64 = 45278215726234785;
pub const CFG_FOOTER: u64 = 14538923454825426;

#[derive(Debug,Clone,Copy)]
pub struct Configuration {
    pub header: u64,
    pub imu0: Option<ImuFeatures>,
    pub imu1: Option<ImuFeatures>,
    pub imu2: Option<ImuFeatures>,
    pub imu3: Option<ImuFeatures>,
    pub mag0: Option<MagFeatures>,
    pub mag1: Option<MagFeatures>,
    pub mag2: Option<MagFeatures>,
    pub mag3: Option<MagFeatures>,
    pub dshot_speed: DshotSpeed,
    pub motor_dir: [bool;4],
    pub attitude_loop_time: Duration,
    pub mav_freq: MavlinkMsgFrequencies,
    pub footer: u64,
}

#[derive(Debug,Clone,Copy)]
pub struct MavlinkMsgFrequencies {
    pub heartbeat: Option<Duration>,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            header: CFG_HEADER,
            imu0: None,
            imu1: None,
            imu2: None,
            imu3: None,
            mag0: None,
            mag1: None,
            mag2: None,
            mag3: None,
            dshot_speed: Default::default(),
            motor_dir: [false;4],
            attitude_loop_time: Duration::from_hz(100),
            mav_freq: MavlinkMsgFrequencies { 
                heartbeat: Some(Duration::from_hz(1)),
            },
            footer: CFG_FOOTER,
        }
    }
}

#[derive(Debug,Clone,Copy)]
pub struct ImuFeatures {
    ///Accelerometer calibration
    pub acc_cal: Option<SimpleCalibration>,
    /// Gyroscope calibration
    pub gyr_cal: Option<SimpleCalibration>,
    /// IMU rotation and translation
    pub imu_ext: Extrinsics,
}

impl Default for ImuFeatures {
    fn default() -> Self {
        ImuFeatures {
            acc_cal: None,
            gyr_cal: None,
            imu_ext: Default::default(),
        }
    }
}

#[derive(Debug,Clone,Copy)]
pub struct MagFeatures {
    ///Magnetometer calibration
    pub mag_cal: Option<SimpleCalibration>,
    /// Magnetometer rotation (XYZ)
    pub mag_ext: Extrinsics,
}

impl Default for MagFeatures {
    fn default() -> Self {
        Self { mag_cal: Default::default(), mag_ext: Default::default() }
    }
}

#[derive(Debug,Clone,Copy)]
pub struct SimpleCalibration {
    pub scale: Vector3<f32>,
    pub offset: Vector3<f32>
}

impl SimpleCalibration {
    pub fn apply(&self,sample: &mut Vector3<f32>) {
        sample.zip_zip_apply(&self.offset, &self.scale, |x,o,s| {
            *x = s*(*x - o)
        });
    }
}

impl Default for SimpleCalibration {
    fn default() -> Self {
        Self { scale: Vector3::new(1., 1., 1.), offset: Vector3::zeros() }
    }
}


#[derive(Debug,Clone,Copy)]
pub struct Extrinsics {
    pub rotation: Matrix3<f32>,
    pub translation: Vector3<f32>
}

impl Default for Extrinsics {
    fn default() -> Self {
        Self { rotation: MAT_IDENTITY, translation: VEC_ZEROS }
    }
}

impl Extrinsics {
    pub fn translate(&self, sample: &mut Vector3<f32>) {
        *sample = self.rotation*(*sample) + self.translation
    }
}


#[derive(Debug,Clone,Copy)]
pub struct AdvancedCalibration {
    pub transform: Matrix3<f32>,
    pub offset: Vector3<f32>
}

impl Default for AdvancedCalibration {
    fn default() -> Self {
        Self { transform: Matrix3::identity(), offset: Vector3::zeros() }
    }
}

#[allow(unused)]
#[derive(Debug,Clone,Copy)]
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

impl DshotSpeed {
    pub fn clk_div(&self) -> (u16, u8) {
        match self {
            DshotSpeed::Dshot150    => (104, 0),
            DshotSpeed::Dshot300    => (52, 0),
            DshotSpeed::Dshot600    => (26, 0),
            DshotSpeed::Dshot1200   => (13, 0),
        }
    }
}

const MAT_NEGATE_YZ: Matrix3<f32> = Matrix3::new(
    1., 0., 0.,
    0., -1., 0.,
    0., 0., -1.
);

const MAT_IDENTITY: Matrix3<f32> = Matrix3::new(
    1., 0., 0.,
    0., 1., 0.,
    0., 0., 1.
);

const VEC_ZEROS: Vector3<f32> = Vector3::new(0.,0.,0.);