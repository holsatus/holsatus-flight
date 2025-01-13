#![no_std]
#![deny(clippy::large_futures)]


// Export the logging macros from either defmt or log
#[macro_use]
#[macro_export]
pub mod logging;

pub mod consts;
pub mod airframe;
pub mod calibration;
pub mod drivers;
pub mod estimators;
pub mod filters;
pub mod health;
pub mod hw_abstraction;
pub mod parsers;
pub mod rc_mapping;
pub mod signals;
pub mod sync;
pub mod tasks;
pub mod types;
pub mod utils;
pub mod geo;
pub mod shell;

pub mod errors;

#[cfg(feature = "mavlink")]
pub mod mavlink;

#[cfg(not(feature = "arch-std"))]
use num_traits::Float;

// Re-exported for implementors
pub use embassy_sync;
pub use embassy_time;
pub use embassy_usb;
pub use embedded_io;
pub use embedded_io_async;
pub use embedded_storage_async;
pub use heapless;
pub use nalgebra;

const MAIN_LOOP_FREQ: usize = 1000;
const ANGLE_LOOP_DIV: usize = 10;
const POS_LOOP_DIV: usize = 100;

const DSHOT_MIN: u16 = 48;
const DSHOT_MAX: u16 = 2047;

const NUM_IMU: usize = 2;
const NUM_MAG: usize = 2;
const MOTOR_TIMEOUT_MS: u8 = 100;

#[macro_export]
macro_rules! get_or_warn {
    ($rcv:ident) => {
        async {
            loop {
                use embassy_time::{Duration, with_timeout};
                match with_timeout(Duration::from_secs(1), $rcv.get()).await {
                    Ok(value) => break value,
                    Err(_) => warn!("{}: Awaiting value for <{}>", ID, stringify!($rcv)),
                }
            }
        }
    };
}
