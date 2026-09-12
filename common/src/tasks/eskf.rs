use crate::{
    consts::GRAVITY,
    filters::IntegratingComplementary,
    signals as s,
    sync::{
        channel::Channel,
        watch::{Sender, Watch},
    },
    types::measurements::{BarometerData, Imu6DofData, ViconData},
};
use embassy_executor::SendSpawner;
use embassy_time::{Duration, Instant};
use nalgebra::{SMatrix, UnitQuaternion, Vector3};

#[allow(unused_imports)]
use num_traits::Float as _;
use num_traits::Zero;

#[cfg(feature = "gnss")]
use crate::types::measurements::GnssData;

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

pub enum Message {
    ImuData(crate::types::measurements::Imu6DofData<f32>),
    ViconData(crate::types::measurements::ViconData),
    BaroData(crate::types::measurements::BarometerData),

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
        #[param(rename = "baro_noise")]
        pub baro_noise_std: f32,
    }

    crate::const_default!(
        Parameters => {
            acc_noise_std: 0.002,
            gyr_noise_std: 0.001,
            acc_drift_std: 0.0001,
            gyr_drift_std: 0.0001,
            covariance_init: 0.1,
            baro_noise_std: 0.5,
        }
    );

    crate::param_table!(pub(crate) static TABLE = "eskf" for Parameters);
}

/// Barometric pressure to altitude conversion using the International Standard
/// Atmosphere (ISA).
mod atmosphere {
    #[allow(unused_imports)]
    use num_traits::Float as _;

    use crate::consts::GRAVITY;

    /// ISA sea-level temperature [K]
    const SEA_LEVEL_TEMPERATURE: f32 = 288.15;
    /// ISA temperature lapse rate [K/m]
    const LAPSE_RATE: f32 = 0.0065;
    /// Universal gas constant [J/(mol*K)]
    const GAS_CONSTANT: f32 = 8.314_462_618;
    /// Molar mass of dry air [kg/mol]
    const MOLAR_MASS_AIR: f32 = 0.028_964_4;

    /// Exponent `R * L / (g * M)` of the barometric formula.
    const EXPONENT: f32 = GAS_CONSTANT * LAPSE_RATE / (GRAVITY * MOLAR_MASS_AIR);

    /// Convert a static pressure reading into a height above a reference
    /// pressure using the barometric formula:
    ///
    /// ```text
    /// h = (T0 / L) * (1 - (P / P_ref)^(R * L / (g * M)))
    /// ```
    ///
    /// Passing the pressure measured at the local origin as `reference_pa`
    /// makes the result an altitude relative to that origin. This is the
    /// correct thing to do for a local NED frame, since the absolute sea-level
    /// pressure is unknown and its variation with the weather would otherwise
    /// bias the altitude.
    pub fn altitude_above(pressure_pa: f32, reference_pa: f32) -> f32 {
        (SEA_LEVEL_TEMPERATURE / LAPSE_RATE) * (1.0 - (pressure_pa / reference_pa).powf(EXPONENT))
    }
}

#[cfg(test)]
mod baro_tests {
    use super::atmosphere::altitude_above;
    use approx::assert_relative_eq;

    #[test]
    fn altitude_is_zero_at_reference_pressure() {
        assert_relative_eq!(altitude_above(101_325.0, 101_325.0), 0.0, epsilon = 1e-3);
    }

    #[test]
    fn altitude_follows_pressure_difference() {
        let reference = 101_325.0;
        let above = altitude_above(reference - 100.0, reference);
        let below = altitude_above(reference + 100.0, reference);

        assert!(above > 0.0);
        assert!(below < 0.0);
        // The barometric relation is exponential, so equal and opposite
        // pressure offsets are only approximately symmetric.
        assert_relative_eq!(above, -below, epsilon = 1e-2);
        // Near sea level, 100 Pa corresponds to roughly 8.4 m.
        assert_relative_eq!(above, 8.4, epsilon = 0.3);
    }

    #[test]
    fn altitude_is_scale_invariant() {
        // The conversion is relative, so scaling both pressures equally must
        // yield the same altitude.
        let a = altitude_above(90_000.0, 100_000.0);
        let b = altitude_above(180_000.0, 200_000.0);
        assert_relative_eq!(a, b, epsilon = 1e-3);
    }
}

#[embassy_executor::task]
pub async fn main() -> ! {
    Eskf::new().await.run().await
}

/// The single navigation filter task.
///
/// Owns the [`eskf_rs::NavigationFilter`] and translates the asynchronous
/// [`Message`] stream into filter predictions and observation updates.
struct Eskf<'a> {
    filter: eskf_rs::NavigationFilter,
    snd_estimate: Sender<'a, EskfEstimate>,

    last_imu_time: Instant,
    last_vicon_time: Instant,
    comps: [IntegratingComplementary<f32>; 3],
    position_valid: bool,

    /// Pressure captured at the local origin. `None` until the first valid
    /// barometric sample is received.
    baro_reference_pa: Option<f32>,
    /// Vertical offset applied to the barometric altitude so that it shares the
    /// local origin with the other vertical sources.
    baro_offset_m: f32,
    /// Variance of the barometer noise measurements.
    baro_noise_var: f32,

    #[cfg(feature = "gnss")]
    gnss_fusion: gnss_fusion::EskfGnssFusion,
}

impl Eskf<'_> {
    async fn new() -> Self {
        let params = params::TABLE.read().await;

        let spawner = SendSpawner::for_current_executor().await;
        if imu_helper().map(|task| spawner.spawn(task)).is_err() {
            error!("[eskf] Failed to spawn imu_helper task");
        }

        if vicon_helper().map(|task| spawner.spawn(task)).is_err() {
            error!("[eskf] Failed to spawn vicon_helper task");
        }

        #[cfg(feature = "gnss")]
        if gnss_helper().map(|task| spawner.spawn(task)).is_err() {
            error!("[eskf] Failed to spawn gnss_helper task");
        }

        let dt = 1.0 / crate::get_ctrl_freq!() as f32;

        let filter = eskf_rs::NavigationFilter::new()
            .acc_noise_density(params.acc_noise_std)
            .acc_bias_random_walk(params.acc_drift_std)
            .gyr_noise_density(params.gyr_noise_std)
            .gyr_bias_random_walk(params.gyr_drift_std)
            // .covariance_diag(params.covariance_init)
            .with_gravity(Vector3::z() * GRAVITY);

        let baro_noise_var = params.baro_noise_std.powi(2);
        drop(params);

        Self {
            filter,
            snd_estimate: s::ESKF_ESTIMATE.sender(),
            last_imu_time: Instant::MIN,
            last_vicon_time: Instant::MIN,
            comps: [
                IntegratingComplementary::new(0.5, dt),
                IntegratingComplementary::new(0.5, dt),
                IntegratingComplementary::new(0.5, dt),
            ],
            position_valid: false,
            baro_reference_pa: None,
            baro_offset_m: 0.0,
            baro_noise_var,
            #[cfg(feature = "gnss")]
            gnss_fusion: gnss_fusion::EskfGnssFusion::new(),
        }
    }

    async fn run(&mut self) -> ! {
        info!("[eskf] Entering main loop");
        loop {
            let message = CHANNEL.receive().await;
            self.handle_message(message);
        }
    }

    fn handle_message(&mut self, message: Message) {
        match message {
            Message::ImuData(imu_data) => self.on_imu_data(imu_data),
            Message::ViconData(vicon_data) => self.on_vicon_data(vicon_data),
            Message::BaroData(baro_data) => self.on_baro_data(baro_data),
            #[cfg(feature = "gnss")]
            Message::GnssData(gnss_data) => self.on_gnss_data(gnss_data),
            #[cfg(feature = "gnss")]
            Message::GnssResetOrigin => self.gnss_fusion.reset_origin(),
        }
    }

    /// NOTE: Currently *every* IMU sample is used directly. It might be
    /// adequate to average a few samples.
    fn on_imu_data(&mut self, imu_data: Imu6DofData<f32>) {
        // Calculate the delta time in f32 seconds (prevent delta from going to zero)
        let timestamp = Instant::from_micros(imu_data.timestamp_us);
        let delta_dur = timestamp.saturating_duration_since(self.last_imu_time);
        let delta_time = delta_dur.as_micros() as f32 * 1e-6;
        self.last_imu_time = timestamp;

        if delta_time.is_zero() {
            error!("[eskf] Non-positive IMU measurement delta time");
            return;
        }

        let time_now = Instant::now();

        let mut position_provider = false;

        position_provider |= time_now.duration_since(self.last_vicon_time) < Duration::from_secs(2);

        #[cfg(feature = "gnss")]
        {
            position_provider |= self.gnss_fusion.time_elapsed() < Duration::from_secs(2);
        }

        if self.position_valid && !position_provider {
            warn!("[eskf] No position provider, invalidating position");
            self.position_valid = false;
        }

        // Meat and potatos
        self.filter
            .predict(imu_data.acc.into(), imu_data.gyr.into(), delta_time);

        // Using the latest delta time
        self.comps.iter_mut().for_each(|c| c.set_dt(delta_time));

        // Complementary filter for smoothing out corrections
        let smooth_position = [
            self.comps[0].update(self.filter.position[0], self.filter.velocity[0]),
            self.comps[1].update(self.filter.position[1], self.filter.velocity[1]),
            self.comps[2].update(self.filter.position[2], self.filter.velocity[2]),
        ];

        // Without a global position reference the horizontal position and
        // velocity are unobservable, so pin them to the local origin instead of
        // integrating them without bound. The vertical channel is only pinned
        // while no barometer is available; once one is, altitude is observable
        // and must be left to the filter. Biases are deliberately not reset, so
        // that they can converge from whichever observations exist.
        if !self.position_valid {
            self.filter.position.x = 0.0;
            self.filter.position.y = 0.0;
            self.filter.velocity.x = 0.0;
            self.filter.velocity.y = 0.0;

            if self.baro_reference_pa.is_none() {
                self.filter.position.z = 0.0;
                self.filter.velocity.z = 0.0;
            }

            #[cfg(feature = "gnss")]
            {
                self.gnss_fusion.reset_origin();
            }
        }

        let estimate = EskfEstimate {
            pos: smooth_position.into(),
            vel: self.filter.velocity,
            att: self.filter.rotation,
            gyr_bias: self.filter.gyr_bias,
            acc_bias: self.filter.acc_bias,
        };

        self.snd_estimate.send(estimate);
    }

    fn on_vicon_data(&mut self, vicon_data: ViconData) {
        // TODO Skip outliers / high variance?

        // We do not need to process these too rapidly, 10 hz like the average GPS
        let time_now = Instant::now();
        if time_now.duration_since(self.last_vicon_time) < Duration::from_millis(100) {
            return;
        }

        self.last_vicon_time = time_now;

        let position = vicon_data.position.into();

        let roll = vicon_data.attitude[0];
        let pitch = vicon_data.attitude[1];
        let yaw = vicon_data.attitude[2];

        // For simulations, since it uses more common euler angle order
        let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

        // Convert the variance data into matrices
        let position_var = SMatrix::from_array_storage(nalgebra::ArrayStorage(vicon_data.pos_var));
        let rotation_var = SMatrix::from_array_storage(nalgebra::ArrayStorage(vicon_data.att_var));

        if self
            .filter
            .observe_position(position, position_var)
            .is_err()
        {
            error!("[eskf] Unable to do matrix inversion during ESKF position update");
        }

        if self
            .filter
            .observe_rotation(rotation, rotation_var)
            .is_err()
        {
            error!("[eskf] Unable to do matrix inversion during ESKF update");
        }

        self.position_valid = true;
    }

    fn on_baro_data(&mut self, baro_data: BarometerData) {
        let pressure_pa = baro_data.pressure_pa;

        // Reject obviously bad samples so that a single glitch cannot poison the
        // reference pressure or the filter.
        if !pressure_pa.is_finite() || pressure_pa <= 0.0 {
            warn!(
                "[eskf] Ignoring invalid barometric pressure: {}",
                pressure_pa
            );
            return;
        }

        // Initialise the local vertical datum from the first valid reading, so
        // that altitude is zero at the origin. This should ideally happen while
        // the vehicle is stationary on the ground.
        let reference_pa = match self.baro_reference_pa {
            Some(reference_pa) => reference_pa,
            None => {
                // The first barometric sample normally establishes the vertical
                // datum. If a GNSS-derived origin already exists, align the
                // barometer to it instead so that both sources agree on where
                // `z = 0` is.
                self.baro_offset_m = self.filter.position.z;
                info!(
                    "[eskf] Initialised barometric altitude at {} Pa (origin offset {} m)",
                    pressure_pa, self.baro_offset_m
                );
                self.baro_reference_pa = Some(pressure_pa);
                pressure_pa
            }
        };

        let altitude_m = atmosphere::altitude_above(pressure_pa, reference_pa);

        // NED has z pointing down, so the local down coordinate is the negated
        // altitude, shifted by any origin offset.
        if self
            .filter
            .observe_position_z(self.baro_offset_m - altitude_m, self.baro_noise_var)
            .is_err()
        {
            error!("[eskf] Unable to do matrix inversion during barometer update");
        }
    }

    #[cfg(feature = "gnss")]
    fn on_gnss_data(&mut self, gnss_data: GnssData) {
        if self
            .gnss_fusion
            .fuse_measurement(&gnss_data, &mut self.filter)
        {
            self.position_valid = true;
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

    use crate::{
        tasks::eskf::{KM_PER_DEG_OF_LAT, lat_factor},
        types::measurements::GnssData,
    };

    pub struct EskfGnssFusion {
        last_time: Instant,
        /// Horizontal origin (latitude/longitude) of the local NED frame.
        horizontal_origin: Option<(i32, i32)>,
        /// MSL altitude of the local NED frame's origin (`z = 0`). Once
        /// established this is kept across horizontal origin resets, so losing
        /// and regaining GNSS cannot shift the vertical datum.
        origin_altitude_msl: Option<f32>,
    }

    impl EskfGnssFusion {
        pub fn new() -> Self {
            Self {
                last_time: Instant::MIN,
                horizontal_origin: None,
                origin_altitude_msl: None,
            }
        }

        /// Re-zero the horizontal origin at the next fix. The vertical datum is
        /// intentionally preserved, since it is normally pinned by the
        /// barometer before GNSS ever locks.
        pub fn reset_origin(&mut self) {
            self.horizontal_origin = None
        }

        pub fn time_elapsed(&self) -> Duration {
            self.last_time.elapsed()
        }

        pub fn fuse_measurement(
            &mut self,
            gnss_data: &GnssData,
            filter: &mut NavigationFilter,
        ) -> bool {
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
            let altitude_msl = gnss_data.height_above_msl;

            // Horizontal origin: latitude/longitude only. The vertical datum is
            // owned by the barometer, so GNSS must not redefine it.
            let (origin_lat, origin_lon) =
                *self.horizontal_origin.get_or_insert((lat_raw, lon_raw));

            let north_delta = KM_PER_DEG_OF_LAT * (lat_raw - origin_lat) as f32 * 1e-7 * 1e3;
            let east_delta = lat_factor(lat_raw) * (lon_raw - origin_lon) as f32 * 1e-7 * 1e3;

            // The filter's vertical state is already relative to the local
            // origin (and barometer-aided when available), so the MSL altitude
            // of the origin follows from `h_msl + z`. This lets GNSS altitude
            // join the barometer's datum rather than resetting it.
            let origin_altitude_msl = match self.origin_altitude_msl {
                Some(origin_altitude_msl) => origin_altitude_msl,
                None => {
                    let origin_altitude_msl = altitude_msl + filter.position.z;
                    info!(
                        "[eskf] Local origin MSL altitude: {} m",
                        origin_altitude_msl
                    );
                    self.origin_altitude_msl = Some(origin_altitude_msl);
                    origin_altitude_msl
                }
            };

            let down_delta = -(altitude_msl - origin_altitude_msl);

            // Which is our current position, relative to origin
            let position = Vector3::new(north_delta, east_delta, down_delta);

            // Velocity is already in NED coordinates!
            let velocity = Vector3::new(
                gnss_data.velocity_north,
                gnss_data.velocity_east,
                gnss_data.velocity_down,
            );

            if gnss_data.fix == GnssFix::Fix3D {
                let position_var = Self::gnss_position_cov_3d(&gnss_data);
                let velocity_var = Self::gnss_velocity_cov_3d(&gnss_data);

                let pos_res = filter.observe_position(position, position_var);
                let vel_res = filter.observe_velocity(velocity, velocity_var);

                if pos_res.is_err() {
                    error!("[eskf] Compute error during 3D position observation");
                }

                if vel_res.is_err() {
                    error!("[eskf] Compute error during 3D velocity observation");
                }

                vel_res.is_ok() && vel_res.is_ok()
            } else {
                let position_var = Self::gnss_position_cov_2d(&gnss_data);
                let velocity_var = Self::gnss_velocity_cov_2d(&gnss_data);

                let pos_res = filter.observe_position_xy(position.xy(), position_var);
                let vel_res = filter.observe_velocity_xy(velocity.xy(), velocity_var);

                if pos_res.is_err() {
                    error!("[eskf] Compute error during 2D position observation");
                }

                if vel_res.is_err() {
                    error!("[eskf] Compute error during 2D velocity observation");
                }

                vel_res.is_ok() && vel_res.is_ok()
            }
        }

        pub fn gnss_velocity_cov_3d(gnss_data: &GnssData) -> SMatrix<f32, 3, 3> {
            let speed = gnss_data.ground_speed;
            let heading = gnss_data.heading_motion;

            // Use a small minimum sigma to ensure numerical stability if accuracy is reported as 0.0
            let sigma_speed = gnss_data.ground_speed_accuracy.max(0.05);
            let sigma_heading = gnss_data.heading_accuracy.max(0.01);

            // Calculate baseline variance, which also is the along-track variance.
            let sigma_speed_sq = sigma_speed.powi(2);
            let sigma_along_sq = sigma_speed_sq;

            // Cross-track variance is the sum of the baseline isotropic variance and
            // the variance induced by heading uncertainty.
            let sigma_cross_sq = sigma_speed_sq + (speed * sigma_heading).powi(2);

            // Rotate variances into the North-East frame
            let (s, c) = heading.sin_cos();
            let var_vn = c * c * sigma_along_sq + s * s * sigma_cross_sq;
            let var_ve = s * s * sigma_along_sq + c * c * sigma_cross_sq;

            // North-East covariance term
            let cov_vn_ve = c * s * (sigma_along_sq - sigma_cross_sq);

            // Assume the vertical velocity uncertainty is some multiples larger
            // than the ground speed uncertainty. Let barometer handle the rest.
            const VERT_INFLATION: f32 = 4.0;
            let var_vd = VERT_INFLATION * sigma_speed_sq;

            nalgebra::matrix![
                var_vn, cov_vn_ve, 0.0;
                cov_vn_ve, var_ve, 0.0;
                0.0, 0.0, var_vd;
            ]
        }

        pub fn gnss_velocity_cov_2d(gnss_data: &GnssData) -> SMatrix<f32, 2, 2> {
            let speed = gnss_data.ground_speed;
            let heading = gnss_data.heading_motion;

            // Use a small minimum sigma to ensure numerical stability if accuracy is reported as 0.0
            let sigma_speed = gnss_data.ground_speed_accuracy.max(0.05);
            let sigma_heading = gnss_data.heading_accuracy.max(0.01);

            // Calculate baseline variance, which also is the along-track variance.
            let sigma_speed_sq = sigma_speed.powi(2);
            let sigma_along_sq = sigma_speed_sq;

            // Cross-track variance is the sum of the baseline isotropic variance and
            // the variance induced by heading uncertainty.
            let sigma_cross_sq = sigma_speed_sq + (speed * sigma_heading).powi(2);

            // Rotate variances into the North-East frame
            let (s, c) = heading.sin_cos();
            let var_vn = c * c * sigma_along_sq + s * s * sigma_cross_sq;
            let var_ve = s * s * sigma_along_sq + c * c * sigma_cross_sq;

            // North-East covariance term
            let cov_vn_ve = c * s * (sigma_along_sq - sigma_cross_sq);

            nalgebra::matrix![
                var_vn, cov_vn_ve;
                cov_vn_ve, var_ve;
            ]
        }

        pub fn gnss_position_cov_3d(gnss_data: &GnssData) -> SMatrix<f32, 3, 3> {
            // Interpret position accuracy as standard deviation
            SMatrix::from_diagonal(
                &([
                    (gnss_data.horizontal_accuracy).powi(2),
                    (gnss_data.horizontal_accuracy).powi(2),
                    (gnss_data.vertical_accuracy).powi(2),
                ])
                .into(),
            )
        }

        pub fn gnss_position_cov_2d(gnss_data: &GnssData) -> SMatrix<f32, 2, 2> {
            // Interpret position accuracy as standard deviation
            SMatrix::from_diagonal(
                &([
                    (gnss_data.horizontal_accuracy).powi(2),
                    (gnss_data.horizontal_accuracy).powi(2),
                ])
                .into(),
            )
        }
    }
}
