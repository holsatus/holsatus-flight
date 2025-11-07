use crate::{
    consts::GRAVITY,
    signals as s,
    sync::{channel::Channel, watch::Watch},
};
use embassy_executor::SendSpawner;
use embassy_time::{Duration, Instant};
use nalgebra::{SMatrix, UnitQuaternion, Vector3};

#[allow(unused_imports)]
use num_traits::Float as _;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Position<Coordinate> {
    /// UAV-local timestamp in micro seconds
    timestamp: u64,

    /// Position in a NED coordinate frame
    position: Coordinate,

    /// Velocity [m/s] in the global NED frame
    velocity: [f32; 3],

    /// Attitude quaternion in global NED frame
    attitude_q: [f32; 4],
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Geodetic {
    /// Degrees of latitude (north/south)
    latitude: f64,

    /// Degrees of longitude (west/east)
    longitude: f64,

    /// Altitude above ellipse (m)
    altitude: f32,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cartesian(
    /// Position in a local NED coordinate frame
    [f32; 3],
);

pub static ESKF_GLOBAL_POS: Watch<Position<Geodetic>> = Watch::new();
pub static ESKF_LOCAL_POS: Watch<Position<Cartesian>> = Watch::new();

#[embassy_executor::task]
pub async fn imu_buffer_helper() -> ! {
    let mut rcv_imu_data = s::CAL_MULTI_IMU_DATA[0].receiver();
    loop {
        let imu_data = rcv_imu_data.changed().await;
        CHANNEL.send(Message::ImuData(imu_data)).await;
    }
}

pub enum Message {
    ImuData(crate::types::measurements::Imu6DofData<f32>),
    ViconData(crate::types::measurements::ViconData),

    #[cfg(feature = "gnss")]
    GnssData(crate::types::measurements::GnssData),
}

// Do not queue up sensor readings.
static CHANNEL: Channel<Message, 1> = Channel::new();

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EskfEstimate {
    pub pos: Vector3<f32>,
    pub vel: Vector3<f32>,
    pub att: UnitQuaternion<f32>,
    pub gyr_bias: Vector3<f32>,
    pub acc_bias: Vector3<f32>,
}

#[embassy_executor::task]
pub async fn imu_helper() -> ! {
    let mut rcv_imu_data = s::CAL_MULTI_IMU_DATA[0].receiver();
    loop {
        let imu_data = rcv_imu_data.changed().await;
        _ = CHANNEL.try_send(Message::ImuData(imu_data));
    }
}

#[embassy_executor::task]
pub async fn vicon_helper() -> ! {
    let mut rcv_vicon_data = s::VICON_POSITION_ESTIMATE.receiver();
    loop {
        let imu_data = rcv_vicon_data.changed().await;
        CHANNEL.send(Message::ViconData(imu_data)).await;
    }
}

#[cfg(feature = "gnss")]
#[embassy_executor::task]
pub async fn gnss_helper() -> ! {
    let mut rcv_gnss_data = s::RAW_GNSS_DATA.receiver();
    loop {
        let gnss_data = rcv_gnss_data.changed().await;
        CHANNEL.send(Message::GnssData(gnss_data)).await;
    }
}

mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct Parameters {
        #[param(rename = "acc_noise")]
        pub acc_noise_std: f32,
        #[param(rename = "gyr_noise")]
        pub gyr_noise_std: f32,
        #[param(rename = "acc_drift")]
        pub acc_drift_std: f32,
        #[param(rename = "gyr_drift")]
        pub gyr_drift_std: f32,
        #[param(rename = "cov_init")]
        pub covariance_init: f32,
    }

    crate::const_default!(
        Parameters => {
            acc_noise_std: 0.1,
            gyr_noise_std: 0.01,
            acc_drift_std: 0.0001,
            gyr_drift_std: 0.0001,
            covariance_init: 0.1,
        }
    );

    pub(crate) static TABLE: Table<Parameters> = Table::default("eskf");
}

#[cfg(feature = "gnss")]
#[derive(Debug, Clone, Copy, PartialEq)]
struct GnssPoint {
    lat_raw: i32,
    lon_raw: i32,
    altitude: f32,
}

#[cfg(feature = "gnss")]
#[derive(Debug, Clone, Copy, PartialEq)]
struct GnssDelta {
    lat_delta: f32,
    lon_delta: f32,
    alt_delta: f32,
}

#[cfg(feature = "gnss")]
impl core::ops::Sub for GnssPoint {
    type Output = GnssDelta;

    fn sub(self, rhs: Self) -> Self::Output {
        GnssDelta {
            lat_delta: (self.lat_raw - rhs.lat_raw) as f32 * 1e-7,
            lon_delta: (self.lon_raw - rhs.lon_raw) as f32 * 1e-7,
            alt_delta: self.altitude - rhs.altitude,
        }
    }
}

#[embassy_executor::task]
pub async fn main() -> ! {
    info!("[eskf] Task started");

    let params = params::TABLE.read().await;

    let rcv_channel = CHANNEL.receiver();

    let spawner = SendSpawner::for_current_executor().await;
    spawner.spawn(imu_helper().unwrap());
    spawner.spawn(vicon_helper().unwrap());

    #[cfg(feature = "gnss")]
    spawner.spawn(gnss_helper().unwrap());

    let mut snd_eskf_estimate = s::ESKF_ESTIMATE.sender();

    let mut filter = eskf::ESKF::new().with_mut(|filt| {
        filt.acc_noise_std(params.acc_noise_std);
        filt.acc_bias_std(params.acc_drift_std);
        filt.gyr_noise_std(params.gyr_noise_std);
        filt.gyr_bias_std(params.gyr_drift_std);
        filt.covariance_diag(params.covariance_init);
        filt.with_gravity(Vector3::z() * GRAVITY);
    });

    drop(params);

    let mut last_imu_time = Instant::MIN;
    let mut last_vicon_time = Instant::MIN;
    let dt = 1.0 / crate::get_ctrl_freq!() as f32;

    #[cfg(feature = "gnss")]
    let mut gnss_origin: Option<GnssPoint> = None;
    
    // For smoothing the position estimate using the velocity estimate
    let mut comps = [
        crate::filters::IntegratingComplementary::new(0.5, dt),
        crate::filters::IntegratingComplementary::new(0.5, dt),
        crate::filters::IntegratingComplementary::new(0.5, dt),
    ];

    info!("[eskf] Entering main loop");
    loop {
        match rcv_channel.receive().await {
            Message::ImuData(imu_data) => {
                // Calculate the delta time in f32 seconds (prevent delta from going to zero)
                let timestamp = Instant::from_micros(imu_data.timestamp_us);
                let delta_dur = timestamp.duration_since(last_imu_time);
                let delta_time = delta_dur.as_micros() as f32 * 1e-6;
                last_imu_time = timestamp;

                // Prediction takes ~ 4500 us on stm32f405
                filter.predict_optimized(imu_data.acc.into(), imu_data.gyr.into(), delta_time);

                // Using the latest delta time
                comps.iter_mut().for_each(|c| c.set_dt(delta_time));

                // Complementary filter for smoothing out corrections
                let smooth_position = [
                    comps[0].update(filter.position[0], filter.velocity[0]),
                    comps[1].update(filter.position[1], filter.velocity[1]),
                    comps[2].update(filter.position[2], filter.velocity[2]),
                ];

                let estimate = EskfEstimate {
                    pos: smooth_position.into(),
                    vel: filter.velocity,
                    att: filter.rotation,
                    gyr_bias: filter.gyr_bias,
                    acc_bias: filter.acc_bias,
                };

                snd_eskf_estimate.send(estimate);
            }
            Message::ViconData(vicon_data) => {
                // TODO Skip outliers / high variance?

                // We do not need to process these too rapidly, 10 hz like the average GPS
                let time_now = Instant::now();
                if time_now.duration_since(last_vicon_time) < Duration::from_millis(100) {
                    continue;
                }

                last_vicon_time = time_now;

                let position = vicon_data.position.into();

                let roll = vicon_data.attitude[0];
                let pitch = vicon_data.attitude[1];
                let yaw = vicon_data.attitude[2];

                // For simulations, since it uses more common euler angle order
                let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

                // Convert the variance data into matrices
                let position_var =
                    SMatrix::from_array_storage(nalgebra::ArrayStorage(vicon_data.pos_var));
                let rotation_var =
                    SMatrix::from_array_storage(nalgebra::ArrayStorage(vicon_data.att_var));

                let result = filter.observe_position_rotation(
                    position,
                    position_var,
                    rotation,
                    rotation_var,
                );

                if result.is_err() {
                    error!("[eskf] Unable to do matrix inversion during ESKF update");
                }
            },
            #[cfg(feature = "gnss")]
            Message::GnssData(gnss_data) => {
                // TODO Skip outliers / high variance?

                let lat_raw = gnss_data.lat_raw;
                let lon_raw = gnss_data.lon_raw;
                let altitude = gnss_data.altitude;

                // Get the current origin point, initializing if necesary     
                let point = GnssPoint { lat_raw, lon_raw, altitude };           
                let origin = gnss_origin.get_or_insert_with(||point);
                
                let delta = point - *origin;

                // Convert coordinates into equivalent meters
                let lat_delta_meters = KM_PER_DEG_OF_LAT * delta.lat_delta * 1e3;
                let lon_delta_meters = lat_factor(lat_raw) * delta.lon_delta * 1e3;
                let alt_delta_meters = -delta.alt_delta;

                // Which is our current position, relative to origin
                let position = nalgebra::Point3::new(
                    lat_delta_meters,
                    lon_delta_meters,
                    alt_delta_meters
                );

                // Already in NED coordinates!
                let velocity = Vector3::new(
                    gnss_data.vel_north,
                    gnss_data.vel_east,
                    gnss_data.vel_down,
                );

                // Interpret position accuracy as standard deviation
                let pos_variance = Vector3::new(
                    gnss_data.horiz_accuracy.powi(2),
                    gnss_data.horiz_accuracy.powi(2),
                    gnss_data.vert_accuracy.powi(2),
                );

                // The full 3D position variance matrix
                let position_var = SMatrix::from_diagonal(&pos_variance);

                // For the velocity variance we first need to convert 
                // the accuracy estimates from the default polar coordinates
                // consisting of heading and velocity estimates, into seperate
                // cartesian accuracies. This also gives off-axis covariances.

                // The jacobian allows us to transform the polar coordinates
                let heading = gnss_data.heading.to_radians();
                let (heading_sin,heading_cos) = heading.sin_cos();
                let jacobian = nalgebra::Matrix2::new(
                    heading_cos, -gnss_data.vel_ground * heading_sin,
                    heading_sin, gnss_data.vel_ground * heading_cos,
                );

                // Variance in polar coordinates
                let vel_polar_var = nalgebra::Matrix2::new(
                    gnss_data.vel_accuracy.powi(2), 0.0,
                    0.0, gnss_data.heading_accuracy.to_radians().powi(2)
                );

                // Variance in cartesian coordinates
                let vel_cartesian_var = jacobian * vel_polar_var * jacobian.transpose();

                // The full 3D velocity variance matrix
                let velocity_var = nalgebra::stack![
                    vel_cartesian_var, 0;
                    0, nalgebra::matrix![gnss_data.vel_accuracy.powi(2)]
                ];

                let result = filter.observe_position_velocity(
                    position,
                    position_var,
                    velocity,
                    velocity_var
                );

                info!("pos_est: {:?}", filter.position.coords.as_slice());
                info!("vel_est: {:?}", filter.velocity.as_slice());

                if result.is_err() {
                    error!("[eskf] Unable to do matrix inversion during ESKF update");
                }
            }
        }
    }
}

/// Circumference of the earth around the equator in kilometers
const EARTH_CIRCUM_KM: f32 = 40075.0;

/// Kilometers per degree of latitude
const KM_PER_DEG_OF_LAT: f32 = EARTH_CIRCUM_KM / 360.0;

/// Calculates kilometers per degree of longitude for a given latitude
pub fn lat_factor(latitude_raw: i32) -> f32 {
    let latitude_deg = (latitude_raw as f32) * 1e-7;
    KM_PER_DEG_OF_LAT * latitude_deg.to_radians().cos()
}
