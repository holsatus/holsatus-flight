use num_traits::Num;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ViconData {
    pub timestamp_us: u64,
    pub position: [f32; 3],
    pub pos_var: [[f32; 3]; 3],
    pub attitude: [f32; 3],
    pub att_var: [[f32; 3]; 3],
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
    /// The device-local time of when the packet was parsed.
    pub timestamp_us: u64,
    /// The received timestamp of the GNSS packet.
    pub time: GnssTime,
    /// The type of GNSS fix at the time of reception.
    pub fix: GnssFix,
    /// The number of satellites visible to the receiver.
    pub num_satellites: u8,

    /// Raw latitude coordinate, in `1e7 * deg`.
    pub latitude_raw: i32,
    /// Raw longitude coordinate, in `1e7 * deg`.
    pub longitude_raw: i32,
    /// Height above mean sea level (MSL), in `m/s`.
    pub height_above_msl: f32,

    /// Accuracy of `latitude_raw` and `longitude_raw`, in `m/s`.
    pub horizontal_accuracy: f32,
    /// Accuracy of `height_above_msl`, in `m/s`.
    pub vertical_accuracy: f32,

    /// Velocity along the north-south axis, in `m/s`.
    pub velocity_north: f32,
    /// Velocity along the east-west axis, in `m/s`.
    pub velocity_east: f32,
    /// Velocity along the down-up axis, in `m/s`.
    pub velocity_down: f32,

    /// Speed of north-east motion, in `m/s`.
    pub ground_speed: f32,
    /// Accuracy of `ground_speed`, in `m/s`.
    pub ground_speed_accuracy: f32,

    /// Heading of north-east motion, in radians.
    pub heading_motion: f32,
    /// Accuracy of `heading_motion`, in radians.
    pub heading_accuracy: f32,
    pub mag_declination: f32,
}

#[derive(Default, Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GnssFix {
    #[default]
    NoFix,
    TimeOnly,
    Fix2D,
    Fix3D,
}

#[cfg(feature = "gnss")]
impl From<ublox::GnssFixType> for GnssFix {
    fn from(fix: ublox::GnssFixType) -> Self {
        match fix {
            ublox::GnssFixType::Fix2D => Self::Fix2D,
            ublox::GnssFixType::Fix3D => Self::Fix3D,
            ublox::GnssFixType::TimeOnlyFix => Self::TimeOnly,
            _ => Self::NoFix,
        }
    }
}

#[cfg(feature = "gnss")]
impl<'a> From<ublox::packets::nav_pvt::proto23::NavPvtRef<'a>> for GnssData {
    fn from(pvt: ublox::packets::nav_pvt::proto23::NavPvtRef<'a>) -> Self {
        let time = GnssTime {
            year: pvt.year(),
            month: pvt.month(),
            day: pvt.day(),
            hour: pvt.hour(),
            min: pvt.min(),
            sec: pvt.sec(),
        };

        GnssData {
            timestamp_us: embassy_time::Instant::now().as_micros(),
            time,
            fix: pvt.fix_type().into(),
            num_satellites: pvt.num_satellites(),
            latitude_raw: pvt.latitude_raw(),
            longitude_raw: pvt.longitude_raw(),
            height_above_msl: pvt.height_msl() as f32,
            horizontal_accuracy: pvt.horizontal_accuracy() as f32,
            vertical_accuracy: pvt.vertical_accuracy() as f32,
            velocity_north: pvt.vel_north() as f32,
            velocity_east: pvt.vel_east() as f32,
            velocity_down: pvt.vel_down() as f32,
            ground_speed: pvt.ground_speed_2d() as f32,
            ground_speed_accuracy: pvt.speed_accuracy() as f32,
            heading_motion: (pvt.heading_motion() as f32).to_radians(),
            heading_accuracy: (pvt.heading_accuracy() as f32).to_radians(),
            mag_declination: pvt.magnetic_declination() as f32,
        }
    }
}
