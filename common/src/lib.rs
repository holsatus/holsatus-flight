#![no_std]
#![deny(clippy::large_futures)]

// Export the logging macros for either defmt or log
#[macro_use]
#[macro_export]
pub mod logging;

pub mod airframe;
pub mod calibration;
pub mod consts;
pub mod drivers;
pub mod estimators;
pub mod filters;
pub mod geo;
pub mod health;
pub mod hw_abstraction;
pub mod parsers;
pub mod signals;
pub mod sync;
pub mod serial;
pub mod tasks;
pub mod types;
pub mod utils;

pub mod shell;

pub mod errors;

#[allow(unused)]
#[cfg(not(feature = "arch-std"))]
use num_traits::Float as _;

// Re-exported for implementors
pub use embassy_futures;
pub use embassy_sync;
pub use embassy_time;
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
pub mod mavlink2;

const DSHOT_MIN: u16 = 48;
const DSHOT_MAX: u16 = 2047;

const NUM_IMU: usize = 2;
const NUM_MAG: usize = 2;

const MAX_IO_STREAMS: usize = 6;

#[macro_export]
macro_rules! get_or_warn {
    ($rcv:ident) => {
        async {
            loop {
                use embassy_time::{with_timeout, Duration};
                match with_timeout(Duration::from_secs(1), $rcv.get()).await {
                    Ok(value) => break value,
                    Err(_) => trace!("{}: Awaiting value for <{}>", ID, stringify!($rcv)),
                }
            }
        }
    };
}

#[macro_export]
macro_rules! const_default {
    ($type:ty => { $($token:tt)+ } ) => {
        impl $crate::ConstDefault for $type {
            const DEFAULT: Self = Self::const_default();
        }

        impl $type {
            pub const fn const_default() -> Self {
                Self { $($token)+ }
            }
        }

        impl Default for $type {
            fn default() -> Self {
                Self::const_default()
            }
        }
    };
    ($type:ty => $($token:tt)+ ) => {
        impl $crate::ConstDefault for $type {
            const DEFAULT: Self = Self::const_default();
        }

        impl $type {
            pub const fn const_default() -> Self {
                $($token)+
            }
        }

        impl Default for $type {
            fn default() -> Self {
                Self::const_default()
            }
        }
    };
}

pub trait ConstDefault {
    const DEFAULT: Self;
}
