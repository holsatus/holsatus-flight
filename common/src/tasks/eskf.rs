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

    #[cfg(feature = "gnss")]
    GnssResetOrigin,
}

// Do not queue up sensor readings.
pub static CHANNEL: Channel<Message, 10> = Channel::new();

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
        CHANNEL.send(Message::ImuData(imu_data)).await;
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
            acc_noise_std: 0.002,
            gyr_noise_std: 0.001,
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

    let mut filter = eskf_rs::NavigationFilter::new()
        .acc_noise_density(params.acc_noise_std)
        .acc_bias_random_walk(params.acc_drift_std)
        .gyr_noise_density(params.gyr_noise_std)
        .gyr_bias_random_walk(params.gyr_drift_std)
        // .covariance_diag(params.covariance_init)
        .with_gravity(Vector3::z() * GRAVITY);

    drop(params);

    let mut last_imu_time = Instant::MIN;
    let mut last_vicon_time = Instant::MIN;
    let dt = 1.0 / crate::get_ctrl_freq!() as f32;

    #[cfg(feature = "gnss")]
    let mut gnss_fusion = gnss_fusion::EskfGnssFusion::new();

    // For smoothing the position estimate using the velocity estimate
    let mut comps = [
        crate::filters::IntegratingComplementary::new(0.5, dt),
        crate::filters::IntegratingComplementary::new(0.5, dt),
        crate::filters::IntegratingComplementary::new(0.5, dt),
    ];

    let mut position_valid = false;

    info!("[eskf] Entering main loop");
    loop {
        match rcv_channel.receive().await {
            // NOTE: Currently *every* IMU sample is used directly. It might be adequate to
            // average a few samples
            Message::ImuData(imu_data) => {
                // Calculate the delta time in f32 seconds (prevent delta from going to zero)
                let timestamp = Instant::from_micros(imu_data.timestamp_us);
                let delta_dur = timestamp.duration_since(last_imu_time);
                let delta_time = delta_dur.as_micros() as f32 * 1e-6;
                last_imu_time = timestamp;

                let time_now = Instant::now();

                let mut position_provider = false;

                position_provider |=
                    time_now.duration_since(last_vicon_time) < Duration::from_secs(2);

                #[cfg(feature = "gnss")]
                {
                    position_provider |=
                        gnss_fusion.time_elapsed() < Duration::from_secs(2);
                }

                if position_valid && !position_provider {
                    warn!("[eskf] No position provider, invalidating position");
                    position_valid = false;
                }

                // Meat and potatos
                filter.predict(imu_data.acc.into(), imu_data.gyr.into(), delta_time);

                // Using the latest delta time
                comps.iter_mut().for_each(|c| c.set_dt(delta_time));

                // Complementary filter for smoothing out corrections
                let smooth_position = [
                    comps[0].update(filter.position[0], filter.velocity[0]),
                    comps[1].update(filter.position[1], filter.velocity[1]),
                    comps[2].update(filter.position[2], filter.velocity[2]),
                ];

                // Reset if position was previously invalid
                if !position_valid {
                    filter.position = [0.0; 3].into();
                    filter.velocity = [0.0; 3].into();
                    filter.acc_bias = [0.0; 3].into();
                    filter.gyr_bias = [0.0; 3].into();
                    #[cfg(feature = "gnss")] {
                        gnss_fusion.reset_origin();
                    }
                }

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

                if filter.observe_position(
                    position,
                    position_var,
                ).is_err() {
                    error!("[eskf] Unable to do matrix inversion during ESKF position update");
                }

                if filter.observe_rotation(
                    rotation,
                    rotation_var,
                ).is_err() {
                    error!("[eskf] Unable to do matrix inversion during ESKF update");
                }

                position_valid = true;
            }
            #[cfg(feature = "gnss")]
            Message::GnssData(gnss_data) => {
                if gnss_fusion.fuse_measurement(&gnss_data, &mut filter) {
                    position_valid = true;
                }
            },
            #[cfg(feature = "gnss")]
            Message::GnssResetOrigin => gnss_fusion.reset_origin(),
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

#[cfg(feature = "gnss")]
mod gnss_fusion {
    use embassy_time::{Duration, Instant};
    use eskf_rs::NavigationFilter;
    use nalgebra::{SMatrix, Vector3};

    #[allow(unused_imports)]
    use num_traits::Float as _;

    use crate::{tasks::eskf::{GnssPoint, KM_PER_DEG_OF_LAT, lat_factor}, types::measurements::GnssData};

    pub struct EskfGnssFusion {
        last_time: Instant,
        origin: Option<GnssPoint>
    }

    impl EskfGnssFusion {
        pub fn new() -> Self {
            Self {
                last_time: Instant::MIN,
                origin: None,
            }
        }

        pub fn reset_origin(&mut self) {
            self.origin = None
        }

        pub fn time_elapsed(&self) -> Duration {
            self.last_time.elapsed()
        }

        pub fn fuse_measurement(&mut self, gnss_data: &GnssData, filter: &mut NavigationFilter) -> bool {
            use crate::types::measurements::GnssFix;
            const GNSS_MIN_NUM_SATELLITES: u8 = 3;

            if (gnss_data.fix as u8) < (GnssFix::Fix2D as u8) {
                warn!("[eskf] GNSS must have at least 2D fix");
                return false;
            }

            if gnss_data.num_satellites < GNSS_MIN_NUM_SATELLITES {
                warn!("[eskf] Too few satellites to fuse GNSS data");
                return false;
            }

            self.last_time = Instant::from_micros(gnss_data.timestamp_us);

            let lat_raw = gnss_data.latitude_raw;
            let lon_raw = gnss_data.longitude_raw;
            let altitude = gnss_data.height_above_msl;

            // Get the current origin point, initializing if necesary
            let point = GnssPoint {
                lat_raw,
                lon_raw,
                altitude,
            };

            // Retrive the origin, or set it as current position
            let origin = *self.origin.get_or_insert(point);

            // Delta degrees (assume small distances for this)
            let delta = point - origin;

            // Convert delta into equivalent meters
            let north_delta = KM_PER_DEG_OF_LAT * delta.lat_delta * 1e3;
            let east_delta = lat_factor(lat_raw) * delta.lon_delta * 1e3;
            let down_delta = -delta.alt_delta;

            // Which is our current position, relative to origin
            let position =
                Vector3::new(north_delta, east_delta, down_delta);

            // Velocity is already in NED coordinates!
            let velocity = Vector3::new(
                gnss_data.velocity_north,
                gnss_data.velocity_east,
                gnss_data.velocity_down,
            );

            // Interpret position accuracy as standard deviation
            let position_var = SMatrix::from_diagonal(&[
                (gnss_data.horizontal_accuracy).powi(2),
                (gnss_data.horizontal_accuracy).powi(2),
                (gnss_data.vertical_accuracy).powi(2),
            ].into());

            // The velocity variance is a bit more complex
            let velocity_var = Self::gnss_velocity_cov(&gnss_data);

            // These two observations together take 120-180 µs on an
            // stm32f405 with optim-level = 3, pretty good id say?
            if filter.observe_position(
                position,
                position_var,
            ).is_err() {
                error!("[eskf] Unable to do matrix inversion during ESKF velocity update");
            }

            if filter.observe_velocity(
                velocity, 
                velocity_var
            ).is_err() {
                error!("[eskf] Unable to do matrix inversion during ESKF velocity update");
            }

            true
        }

        pub fn gnss_velocity_cov(gnss_data: &GnssData) -> SMatrix<f32, 3, 3> {
            let v_gs = gnss_data.ground_speed;
            let psi = gnss_data.heading_motion;
            
            // Use a small minimum sigma to ensure numerical stability if accuracy is reported as 0.0
            let sigma_gs = gnss_data.ground_speed_accuracy.max(1e-3);
            let sigma_psi = gnss_data.heading_accuracy.max(1e-3);

            // Calculate baseline variance, which also is the along-track variance.
            let sigma_gs_sq = sigma_gs.powi(2);
            let sigma_along_sq = sigma_gs_sq;

            // Cross-track variance is the sum of the baseline isotropic variance and
            // the variance induced by heading uncertainty.
            let sigma_cross_sq = sigma_gs_sq + (v_gs * sigma_psi).powi(2);

            // Rotate variances into the North-East frame
            let (s, c) = psi.sin_cos();
            let var_vn = c * c * sigma_along_sq + s * s * sigma_cross_sq;
            let var_ve = s * s * sigma_along_sq + c * c * sigma_cross_sq;

            // North-East covariance term
            let cov_vn_ve = c * s * (sigma_along_sq - sigma_cross_sq);

            // Assume the vertical velocity uncertainty is the same as the horizontal
            // ground speed uncertainty.
            let var_vd = sigma_gs_sq;

            SMatrix::<f32, 3, 3>::new(
                var_vn,    cov_vn_ve, 0.0,
                cov_vn_ve, var_ve,    0.0,
                0.0,       0.0,       var_vd,
            )
        }
    }
}
