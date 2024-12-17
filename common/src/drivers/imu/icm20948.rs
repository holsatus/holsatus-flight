use embedded_hal_async::delay::DelayNs;
use icm20948_async::{self, Icm20948, IcmBusI2c, IcmBusSpi, IcmError, Init, MagEnabled};

use crate::{
    errors::DeviceError, hw_abstraction::{Imu6Dof, Imu9Dof}, types::measurements::{Imu6DofData, Imu9DofData}
};

impl<BUS, MAG, DELAY> Imu6Dof for Icm20948<IcmBusI2c<BUS>, MAG, Init, DELAY>
where
    BUS: embedded_hal_async::i2c::I2c,
    DELAY: DelayNs,
{
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_acc()
            .await
            .map_err(|e|DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_gyr()
            .await
            .map_err(|e|DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        self.read_6dof()
            .await
            .map_err(|e|DeviceError::I2c(e.into()))
            .map(|raw| Imu6DofData {
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
            })
    }
}

impl<BUS, DELAY> Imu9Dof for Icm20948<IcmBusI2c<BUS>, MagEnabled, Init, DELAY>
where
    BUS: embedded_hal_async::i2c::I2c,
    DELAY: DelayNs,
{
    async fn read_mag(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_mag()
            .await
            .map_err(|e|DeviceError::I2c(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr_mag(
        &mut self,
    ) -> Result<crate::types::measurements::Imu9DofData<f32>, DeviceError> {
        self.read_9dof()
            .await
            .map_err(|e|DeviceError::I2c(e.into()))
            .map(|raw| Imu9DofData {
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
                mag: raw.mag.into(),
            })
    }
}


impl<BUS, MAG, DELAY> Imu6Dof for Icm20948<IcmBusSpi<BUS>, MAG, Init, DELAY>
where
    BUS: embedded_hal_async::spi::SpiDevice,
    BUS::Error: embedded_hal::spi::Error,
    DELAY: DelayNs,
{
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_acc()
            .await
            .map_err(|e|DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_gyr()
            .await
            .map_err(|e|DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        self.read_6dof()
            .await
            .map_err(|e|DeviceError::Spi(e.into()))
            .map(|raw| Imu6DofData {
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
            })
    }
}

impl<BUS, DELAY> Imu9Dof for Icm20948<IcmBusSpi<BUS>, MagEnabled, Init, DELAY>
where
    BUS: embedded_hal_async::spi::SpiDevice,
    DELAY: DelayNs,
{
    async fn read_mag(&mut self) -> Result<[f32; 3], DeviceError> {
        self.read_mag()
            .await
            .map_err(|e|DeviceError::Spi(e.into()))
            .map(|x| x.into())
    }

    async fn read_acc_gyr_mag(
        &mut self,
    ) -> Result<crate::types::measurements::Imu9DofData<f32>, DeviceError> {
        self.read_9dof()
            .await
            .map_err(|e|DeviceError::Spi(e.into()))
            .map(|raw| Imu9DofData {
                gyr: raw.gyr.into(),
                acc: raw.acc.into(),
                mag: raw.mag.into(),
            })
    }
}

pub fn from_i2c_err<E: embedded_hal::i2c::Error>(value: IcmError<E>) -> DeviceError {
    match value {
        IcmError::BusError(bus_err) => DeviceError::I2c(bus_err.into()),
        IcmError::ImuSetupError => DeviceError::IdentificationError,
        IcmError::MagSetupError => DeviceError::IdentificationError,
        IcmError::InterruptPinError => DeviceError::ExtInterruptError,
    }
}

pub fn from_spi_err<E: embedded_hal::spi::Error>(value: IcmError<E>) -> DeviceError {
    match value {
        IcmError::BusError(bus_err) => DeviceError::Spi(bus_err.into()),
        IcmError::ImuSetupError => DeviceError::IdentificationError,
        IcmError::MagSetupError => DeviceError::IdentificationError,
        IcmError::InterruptPinError => DeviceError::ExtInterruptError,
    }
}

