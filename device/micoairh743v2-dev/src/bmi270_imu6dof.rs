//! `Imu6Dof` trait implementation for the BMI270 IMU driver.
//!
//! Converts raw i16 LSB values to calibrated f32 SI units.
//!
//! BMI270 default output ranges (factory reset, no explicit range register writes):
//!   Accel: +/- 2 g  -> sensitivity 16384 LSB/g  -> scale = 9.81 / 16384.0 m/s^2 per LSB
//!   Gyro:  +/- 2000 dps -> sensitivity 16.384 LSB/dps -> scale = PI / (180 * 16.384) rad/s per LSB
//!
//! If the firmware later configures a different range (e.g. +/- 8 g -> 4096 LSB/g),
//! update ACC_SCALE accordingly.

use core::f32::consts::PI;

use common::errors::DeviceError;
use common::hw_abstraction::Imu6Dof;
use common::types::measurements::Imu6DofData;
use embassy_time::Instant;
use embedded_hal_async::spi::SpiDevice;

use crate::bmi270::{Bmi270, BMI270_CONFIG_FILE};

// Accel: +/- 2 g, 16384 LSB/g
const ACC_SCALE: f32 = 9.81 / 16384.0;

// Gyro: +/- 2000 dps, 16.384 LSB/dps
const GYR_SCALE: f32 = PI / (180.0 * 16.384);

pub struct Bmi270Imu6Dof<SPI>(Bmi270<SPI>);

impl<SPI> Bmi270Imu6Dof<SPI> {
    pub fn new(inner: Bmi270<SPI>) -> Self {
        Self(inner)
    }
}

impl<SPI: SpiDevice> Bmi270Imu6Dof<SPI> {
    /// Initialize the BMI270 sensor. Must be called before `read_acc_gyr`.
    pub async fn init(&mut self) -> Result<(), crate::bmi270::Error> {
        self.0.init(&BMI270_CONFIG_FILE).await
    }
}

impl<SPI: SpiDevice> Imu6Dof for Bmi270Imu6Dof<SPI> {
    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        let d = self.0.read().await.map_err(|_| DeviceError::IdentificationError)?;
        Ok(Imu6DofData {
            timestamp_us: Instant::now().as_micros(),
            acc: [
                d.accel.x as f32 * ACC_SCALE,
                d.accel.y as f32 * ACC_SCALE,
                d.accel.z as f32 * ACC_SCALE,
            ],
            gyr: [
                d.gyro.x as f32 * GYR_SCALE,
                d.gyro.y as f32 * GYR_SCALE,
                d.gyro.z as f32 * GYR_SCALE,
            ],
        })
    }

    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        Ok(self.read_acc_gyr().await?.acc)
    }

    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        Ok(self.read_acc_gyr().await?.gyr)
    }
}
