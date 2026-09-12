#![no_std]
#![deny(clippy::large_futures)]

// Export the logging macros for either defmt or log
#[macro_use]
#[macro_export]
pub mod logging;

pub mod abstraction;
pub mod actuators;
pub mod airframe;
pub mod calibration;
pub mod consts;
pub mod drivers;
pub mod errors;
pub mod estimators;
pub mod filters;
pub mod geo;
pub mod health;
pub mod multicopter;
pub mod params;
pub mod parsers;
pub mod serial;
pub mod shell;
pub mod signals;
pub mod sync;
pub mod tasks;
pub mod types;
pub mod utils;
pub mod vehicle;
pub mod wrapped;

pub use abstraction::trigger::{OnFalling, OnRising};

#[allow(unused)]
#[cfg(not(feature = "std"))]
use num_traits::Float as _;

// Re-exported for implementors
pub use embassy_futures;
pub use embassy_sync;
pub use embassy_time;
pub use embedded_hal;
pub use embedded_hal_async;
pub use embedded_hal_bus;
pub use embedded_io;
pub use embedded_io_async;
pub use embedded_storage_async;
pub use grantable_io;
pub use heapless;
pub use mav_param;
pub use nalgebra;

#[cfg(feature = "usb")]
pub use embassy_usb;

#[cfg(feature = "mavlink")]
pub mod mavlink;

const DSHOT_MIN: u16 = 48;
const DSHOT_MAX: u16 = 2047;

const NUM_MAG: usize = 2;

const MAX_IO_STREAMS: usize = 6;

#[cfg(not(feature = "imu_count_1"))]
compile_error!("An IMU count feature ('imu_count_1' through 'imu_count_4') must be enabled.");

pub const IMU_COUNT: usize = {
    cfg_select! {
        feature = "imu_count_4" => 4,
        feature = "imu_count_3" => 3,
        feature = "imu_count_2" => 2,
        feature = "imu_count_1" => 1,
    }
};

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ImuIndex {
    #[cfg(feature = "imu_count_1")]
    Imu0 = 0,
    #[cfg(feature = "imu_count_2")]
    Imu1 = 1,
    #[cfg(feature = "imu_count_3")]
    Imu2 = 2,
    #[cfg(feature = "imu_count_4")]
    Imu3 = 3,
}
