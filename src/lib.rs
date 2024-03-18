#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(let_chains)]
#![feature(async_closure)]
#![no_std]

pub const N_IMU: usize = 2;
pub const N_MAG: usize = 2;

pub static I2C_ASYNC: StaticCell<Mutex<ThreadModeRawMutex, crate::bsp::AsyncI2cPeripheral>> = StaticCell::new();
pub type AsyncI2cDevice = I2cDevice<'static, ThreadModeRawMutex, crate::bsp::AsyncI2cPeripheral>;

pub mod common;
pub mod config;
pub mod functions;
pub mod geo;
pub mod sync;
pub mod health;
pub mod messaging;
pub mod sensors;
pub mod transmitter;
pub mod drivers;
pub mod filters;
pub mod airframe;
pub mod board;

pub use board::bsp;

#[cfg(feature = "mavlink")]
pub mod mavlink;

#[cfg(feature = "mavlink")]
pub const MAX_MAV_STREAMS: usize = 5;

#[cfg(feature = "shell")]
pub mod shell;

pub mod constants;



pub mod t_led_blinker;

pub mod t_commander;
pub mod t_arm_checker;
pub mod t_vehicle_state;
pub mod t_attitude_estimator;
pub mod t_attitude_controller;
pub mod t_motor_mixing;
pub mod t_icm20948_driver;
pub mod t_imu_governor;
pub mod t_mag_governor;
pub mod t_motor_governor;
pub mod t_sbus_reader;
pub mod t_status_printer;
pub mod t_gyr_calibration;
pub mod t_acc_calibration;
pub mod t_rc_mapper;
pub mod t_blackbox_logging;
pub mod t_flight_detector;
pub mod t_librarian;

// pub mod main_high_prio;
// pub mod main_low_prio;

pub use ahrs;
pub use embassy_embedded_hal;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
pub use embassy_executor;
pub use embassy_rp;
pub use embassy_sync;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
pub use embassy_time;
pub use icm20948_async;
pub use nalgebra;
pub use sbus;
pub use static_cell;
use static_cell::StaticCell;
