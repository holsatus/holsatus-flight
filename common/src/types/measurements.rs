use num_traits::Num;
use serde::{Deserialize, Serialize};


#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ViconData {
    pub timestamp_us: u64,
    pub quality: u8,
    pub position: [f32; 3],
    pub attitude: [f32; 3],
}


#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Imu6DofData<T: Num> {
    pub timestamp_us: u64,
    pub gyr: [T; 3],
    pub acc: [T; 3],
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Imu9DofData<T: Num> {
    pub timestamp_us: u64,
    pub gyr: [T; 3],
    pub acc: [T; 3],
    pub mag: [T; 3],
}

impl<T: Num> From<Imu9DofData<T>> for Imu6DofData<T> {
    fn from(data: Imu9DofData<T>) -> Self {
        Self {
            timestamp_us: data.timestamp_us,
            gyr: data.gyr,
            acc: data.acc,
        }
    }
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GnssTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GnssData {
    pub timestamp_us: u64,
    pub time: GnssTime,
    pub fix: GnssFix,
    pub satellites: u8,

    pub lat_raw: i32,
    pub lon_raw: i32,
    pub altitude: f32,

    pub horiz_accuracy: f32,
    pub vert_accuracy: f32,

    pub vel_north: f32,
    pub vel_east: f32,
    pub vel_down: f32,

    pub vel_accuracy: f32,

    pub heading: f32,
    pub mag_declination: f32,
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GnssFix {
    #[default]
    NoFix,
    Fix2D,
    Fix3D,
    TimeOnly,
}

impl From<ublox::GpsFix> for GnssFix {
    fn from(fix: ublox::GpsFix) -> Self {
        match fix {
            ublox::GpsFix::Fix2D => Self::Fix2D,
            ublox::GpsFix::Fix3D => Self::Fix3D,
            ublox::GpsFix::TimeOnlyFix => Self::TimeOnly,
            _ => Self::NoFix,
        }
    }
}
