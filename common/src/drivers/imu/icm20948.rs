use embassy_time::Instant;
use icm20948_async::{self, BusI2c, BusSpi, Icm20948, MagEnabled, SetupError};

use crate::{
    errors::DeviceError,
    hw_abstraction::{Imu6Dof, Imu9Dof},
    types::measurements::{Imu6DofData, Imu9DofData},
};

impl<BUS, MAG> Imu6Dof for Icm20948<BusI2c<BUS>, MAG>
where
    BUS: embedded_hal_async::i2c::I2c,
{
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_acc()
            .await
            .map_err(|e| DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_gyr()
            .await
            .map_err(|e| DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        self.read_6dof()
            .await
            .map_err(|e| DeviceError::I2c(e.into()))
            .map(|raw| Imu6DofData {
                timestamp_us: Instant::now().as_micros(),
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
            })
    }
}

impl<BUS> Imu9Dof for Icm20948<BusI2c<BUS>, MagEnabled>
where
    BUS: embedded_hal_async::i2c::I2c,
{
    async fn read_mag(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_mag()
            .await
            .map_err(|e| DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr_mag(
        &mut self,
    ) -> Result<crate::types::measurements::Imu9DofData<f32>, DeviceError> {
        self.read_9dof()
            .await
            .map_err(|e| DeviceError::I2c(e.into()))
            .map(|raw| Imu9DofData {
                timestamp_us: Instant::now().as_micros(),
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
                mag: raw.mag.into(),
            })
    }
}

impl<BUS, MAG> Imu6Dof for Icm20948<BusSpi<BUS>, MAG>
where
    BUS: embedded_hal_async::spi::SpiDevice,
    BUS::Error: embedded_hal::spi::Error,
{
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_acc()
            .await
            .map_err(|e| DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_gyr()
            .await
            .map_err(|e| DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        self.read_6dof()
            .await
            .map_err(|e| DeviceError::Spi(e.into()))
            .map(|raw| Imu6DofData {
                timestamp_us: Instant::now().as_micros(),
                gyr: raw.gyr,
                acc: raw.acc,
            })
    }
}

impl<BUS> Imu9Dof for Icm20948<BusSpi<BUS>, MagEnabled>
where
    BUS: embedded_hal_async::spi::SpiDevice,
{
    async fn read_mag(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_mag()
            .await
            .map_err(|e| DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr_mag(
        &mut self,
    ) -> Result<crate::types::measurements::Imu9DofData<f32>, DeviceError> {
        self.read_9dof()
            .await
            .map_err(|e| DeviceError::Spi(e.into()))
            .map(|raw| Imu9DofData {
                timestamp_us: Instant::now().as_micros(),
                gyr: raw.gyr,
                acc: raw.acc,
                mag: raw.mag,
            })
    }
}

pub fn from_i2c_err<E: embedded_hal::i2c::Error>(value: SetupError<E>) -> DeviceError {
    match value {
        SetupError::Bus(bus_err) => DeviceError::I2c(bus_err.into()),
        SetupError::ImuWhoAmI(_) => DeviceError::IdentificationError,
        SetupError::MagWhoAmI(_) => DeviceError::IdentificationError,
    }
}

pub fn from_spi_err<E: embedded_hal::spi::Error>(value: SetupError<E>) -> DeviceError {
    match value {
        SetupError::Bus(bus_err) => DeviceError::Spi(bus_err.into()),
        SetupError::ImuWhoAmI(_) => DeviceError::IdentificationError,
        SetupError::MagWhoAmI(_) => DeviceError::IdentificationError,
    }
}
