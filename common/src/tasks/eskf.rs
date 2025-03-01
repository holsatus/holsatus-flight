use crate::{consts::GRAVITY, get_ctrl_freq, signals as s, sync::watch::Watch};
use embassy_futures::select::{Either, select};
use embassy_time::Instant;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector2, Vector3};


#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Position<Locality> {
    /// UAV-local timestamp in micro seconds
    timestamp: u64,

    /// Position in a NED coordinate frame
    position: Locality,

    /// Velocity [m/s] in the global NED frame
    velocity: [f32; 3],

    /// Attitude quaternion in global NED frame
    attitude_q: [f32; 4]
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Global {
    /// Degrees of latitude (north/south)
    latitude: f64,

    /// Degrees of longitude (west/east)
    longitude: f64,

    /// Altitude above ellipse (m)
    altitude: f32,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Local (
    /// Position in a local NED coordinate frame
    [f32; 3]
);

pub static ESKF_GLOBAL_POS: Watch<Position<Global>> = Watch::new();
pub static ESKF_LOCAL_POS: Watch<Position<Local>> = Watch::new();

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "position_estimator";
    info!("{}: Task started", STR_ID);

    let mut rcv_imu_data = s::CAL_MULTI_IMU_DATA[0].receiver();
    let mut rcv_gnss_data = s::RAW_GNSS_DATA.receiver();

    let mut snd_estimated_pos = s::ESTIMATED_POS.sender();
    let mut snd_estimated_vel = s::ESTIMATED_VEL.sender();

    let mut filter = eskf::Builder::new()
        .gravity(GRAVITY)
        .acceleration_variance(0.0001)
        .rotation_variance(0.0001)
        .initial_covariance(1.0)
        .build();

    let mut prev_gnss_pos = None;
    let mut curr_gnss_data = None;

    let mut prev_imu_time = Instant::MIN;

    let mut counter = 0;

    info!("{}: Entering main loop", STR_ID);

    let mut acc_accumulator = Vector3::zeros();
    let mut gyr_accumulator = Vector3::zeros();

    loop {
        match select(
            rcv_imu_data.changed(),
            rcv_gnss_data.changed(),
        ).await {

            // IMU update
            Either::First(imu_data) => {


                curr_gnss_data = None;
                let curr_time = Instant::now();
                let delta_time = curr_time.duration_since(prev_imu_time);
                prev_imu_time = curr_time;

                counter += 1;
                acc_accumulator += Vector3::from(imu_data.acc);
                gyr_accumulator += Vector3::from(imu_data.gyr);

                // Divide the loop by 10
                if counter < 10 {
                    continue
                }

                info!("imu_data: {:?}", imu_data);

                if prev_gnss_pos.is_some() {
                    // Prediction takes ~ 4500 us on stm32f405
                    filter.predict(
                        acc_accumulator.unscale(counter as f32),
                        acc_accumulator.unscale(counter as f32),
                        delta_time.as_micros() as f32 / 1e6,
                    );
                }

                counter = 0;
                acc_accumulator = Vector3::zeros();
                gyr_accumulator = Vector3::zeros();

                continue;
            }

            // GNSS update
            Either::Second(gnss_data) => {
                let gnss_data = curr_gnss_data.insert(gnss_data);

                // The gnss_data signal is updated even when there
                // is no lock or low number of sats. Skip if that
                // is the case.
                if gnss_data.satellites < 3 {
                    prev_gnss_pos = None;
                    continue;
                }

                if prev_gnss_pos.is_none() {
                    filter = eskf::Builder::new()
                        .gravity(GRAVITY)
                        .acceleration_variance(0.0001)
                        .rotation_variance(0.0001)
                        .initial_covariance(1.0)
                        .build();
                }

                let prev_gnss_pos = prev_gnss_pos.get_or_insert_with(||{
                    filter.position.z = gnss_data.altitude;
                    Vector2::new(
                        gnss_data.lat_raw,
                        gnss_data.lon_raw,
                    )
                });

                let position = Point3::new(
                    ((gnss_data.lat_raw - prev_gnss_pos.x) as f32 / 1e7) * KM_PER_DEG_OF_LAT * 1e3,
                    ((gnss_data.lon_raw - prev_gnss_pos.y) as f32 / 1e7) * lat_factor(gnss_data.lat_raw) * 1e3,
                    gnss_data.altitude,
                );

                prev_gnss_pos.x = gnss_data.lat_raw;
                prev_gnss_pos.y = gnss_data.lon_raw;

                let position_var = Vector3::new(
                    gnss_data.horiz_accuracy,
                    gnss_data.horiz_accuracy,
                    gnss_data.vert_accuracy,
                );

                let velocity = Vector3::new(
                    gnss_data.vel_north,
                    gnss_data.vel_east,
                    gnss_data.vel_down,
                );

                // info!("Raw \"local\" position x = {}, y = {}", position.x, position.y);

                if filter.observe_position_velocity(
                    position,
                    Matrix3::from_diagonal(&position_var),
                    velocity,
                    Matrix3::from_diagonal_element(gnss_data.vel_accuracy),
                ).is_err() {
                    error!("Unable to do matrix inversion during ESKF update");
                    continue;
                }
            },
        };

        let pos: [f32; 3] = filter.position.into();
        // info!("Estimated \"local\" position: [{}]", pos);

        // Bring estimated filter position back into a global frame
        if let Some(origin) = prev_gnss_pos {

            // Add the origin back in to the estimate
            let longitude = (filter.position.x / (KM_PER_DEG_OF_LAT * 1e3)) as f64 + (origin.x as f64 / 1e7);
            let latitude = (filter.position.y / (lat_factor(origin.x) * 1e3)) as f64 + (origin.x as f64 / 1e7);
            let altitude = filter.position.z;

            let global_pos = Position {
                timestamp: Instant::now().as_micros(),
                position: Global {
                    latitude,
                    longitude,
                    altitude,
                },
                velocity: filter.velocity.into(),
                attitude_q: filter.orientation.coords.into(),
            };

            // info!("Global position estimate: {:#?}\neuler: {:?}", global_pos, filter.orientation.euler_angles());

            ESKF_GLOBAL_POS.sender().send(global_pos);

            if curr_gnss_data.is_some() {
                filter.position.x = 0.0;
                filter.position.y = 0.0;
            }
        }

        snd_estimated_pos.send(filter.position.into());
        snd_estimated_vel.send(filter.velocity.into());
    }
}

/// Circumference of the earth around the equator in kilometers
const EARTH_CIRCUM_KM: f32 = 40075.0;

/// Kilometers per degree of latitude
const KM_PER_DEG_OF_LAT: f32 = EARTH_CIRCUM_KM / 360.0;

/// Calculates kilometers per degree of longitude for a given latitude
pub fn lat_factor(latitude: i32) -> f32 {
    #[allow(unused_imports)]
    use num_traits::Float as _;
	KM_PER_DEG_OF_LAT * (latitude as f32/180.0).cos()
}
