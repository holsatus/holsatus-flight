//! `Imu6Dof` trait implementation for the BMI088 IMU driver.
//!
//! Converts raw i16 LSB values to calibrated f32 SI units.
//!
//! BMI088 configured ranges (set in bmi088.rs init):
//!   Accel: +-6g,     sensitivity 5460.8 LSB/g   (ACC_RANGE_6G = 0x01)
//!   Gyro:  +-2000 dps, sensitivity 16.384 LSB/dps (GYR_RANGE   = 0x00)

use core::f32::consts::PI;

use common::errors::DeviceError;
use common::hw_abstraction::Imu6Dof;
use common::types::measurements::Imu6DofData;
use embassy_time::Instant;
use embedded_hal_async::spi::SpiDevice;

use crate::bmi088::Bmi088;

// Accel: +-6g, 5460.8 LSB/g
const ACC_SCALE: f32 = 9.81 / 5460.8;

// Gyro: +-2000 dps, 16.384 LSB/dps
const GYR_SCALE: f32 = PI / (180.0 * 16.384);

pub struct Bmi088Imu6Dof<ACCEL, GYRO>(Bmi088<ACCEL, GYRO>);

impl<ACCEL, GYRO> Bmi088Imu6Dof<ACCEL, GYRO> {
    pub fn new(inner: Bmi088<ACCEL, GYRO>) -> Self {
        Self(inner)
    }
}

impl<ACCEL: SpiDevice, GYRO: SpiDevice> Bmi088Imu6Dof<ACCEL, GYRO> {
    /// Initialize the BMI088. Must be called before `read_acc_gyr`.
    pub async fn init(&mut self) -> Result<(), crate::bmi088::Error> {
        self.0.init().await
    }
}

impl<ACCEL: SpiDevice, GYRO: SpiDevice> Imu6Dof for Bmi088Imu6Dof<ACCEL, GYRO> {
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
