#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(let_chains)]
#![no_std]

pub mod common;
pub mod config;
pub mod functions;
pub mod geo;
pub mod health;
pub mod mavlink;
pub mod messaging;
pub mod sensors;
pub mod transmitter;
pub mod drivers;
pub mod filters;
pub mod airframe;

pub mod constants;

pub const N_IMU: usize = 2;
pub const N_MAG: usize = 2;

pub mod t_led_blinker;

pub mod t_commander;
pub mod t_arm_checker;
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
pub mod t_config_master;
pub mod t_rc_mapper;
pub mod t_flight_detector;


pub mod main_core0;
pub mod main_core1;

pub use ahrs;
pub use embassy_embedded_hal;
pub use embassy_executor;
pub use embassy_rp;
pub use embassy_sync;
pub use embassy_time;
pub use icm20948_async;
pub use nalgebra;
pub use sbus;
pub use static_cell;
