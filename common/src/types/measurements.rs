use num_traits::Num;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Imu6DofData<T: Num> {
    pub gyr: [T; 3],
    pub acc: [T; 3],
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Imu9DofData<T: Num> {
    pub gyr: [T; 3],
    pub acc: [T; 3],
    pub mag: [T; 3],
}

impl<T: Num> From<Imu9DofData<T>> for Imu6DofData<T> {
    fn from(data: Imu9DofData<T>) -> Self {
        Self {
            gyr: data.gyr,
            acc: data.acc,
        }
    }
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GnssData {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub fix: GnssFix,
    pub sats: u8,
    pub lat_raw: i32,
    pub lon_raw: i32,
    pub altitude: f32,
    pub vel_north: f32,
    pub vel_east: f32,
    pub vel_down: f32,
    pub heading: f32,
    pub accuracy_north: u32,
    pub accuracy_east: u32,
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

impl From<ublox::GpsFix> for GnssFix{
    fn from(fix: ublox::GpsFix) -> Self {
        match fix {
            ublox::GpsFix::Fix2D => Self::Fix2D,
            ublox::GpsFix::Fix3D => Self::Fix3D,
            ublox::GpsFix::TimeOnlyFix => Self::TimeOnly,
            _ => Self::NoFix,
        }
    }
}
