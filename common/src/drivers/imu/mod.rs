pub mod icm20948;
use embassy_time::{Delay, Duration, WithTimeout};
use embedded_hal_async::{i2c::I2c, spi::SpiDevice};
pub use icm20948_async;
use icm20948_async::{Icm20948, Icm20948Config};

use crate::{errors::DeviceError, hw_abstraction::{Imu6Dof, Imu9Dof}};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum ImuConfig {
    Icm20948(Icm20948Config),
}

impl ImuConfig {
    pub async fn i2c_setup_6dof<BUS: I2c>(
        &self,
        i2c: BUS,
        addr: Option<u8>,
    ) -> Result<impl Imu6Dof, DeviceError> {
        match self {
            ImuConfig::Icm20948(config) => {
                let icm = Icm20948::new_i2c_from_cfg(i2c, *config, Delay)
                    .set_address(addr.unwrap_or(0x69));

                // Try to run setup, but with a timeout to detect hanging devices
                let timeout = Duration::from_secs(2);
                icm.initialize_6dof().with_timeout(timeout).await
                    .map_err(|_| DeviceError::Timeout {millis: timeout.as_millis()})?
                    .map_err(icm20948::from_i2c_err)

            }
        }
    }

    pub async fn spi_setup_6dof<BUS: SpiDevice>(
        &self,
        spi: BUS,
    ) -> Result<impl Imu6Dof, DeviceError> {
        match self {
            ImuConfig::Icm20948(config) => {
                let icm = Icm20948::new_spi_from_cfg(spi, *config, Delay);

                // Try to run setup, but with a timeout to detect hanging devices
                let timeout = Duration::from_secs(2);
                icm.initialize_6dof().with_timeout(timeout).await
                    .map_err(|_| DeviceError::Timeout {millis: timeout.as_millis()})?
                    .map_err(icm20948::from_spi_err)
            }
        }
    }

    pub async fn i2c_setup_9dof<BUS: I2c>(
        &self,
        i2c: BUS,
        addr: Option<u8>,
    ) -> Result<impl Imu9Dof, DeviceError> {
        match self {
            ImuConfig::Icm20948(config) => {
                let icm = Icm20948::new_i2c_from_cfg(i2c, *config, Delay)
                    .set_address(addr.unwrap_or(0x69));

                // Try to run setup, but with a timeout to detect hanging devices
                let timeout = Duration::from_secs(2);
                icm.initialize_9dof().with_timeout(timeout).await
                    .map_err(|_| DeviceError::Timeout {millis: timeout.as_millis()})?
                    .map_err(icm20948::from_i2c_err)
            }
        }
    }

    pub async fn spi_setup_9dof<BUS: SpiDevice<Error = embedded_hal::spi::ErrorKind>>(
        &self,
        spi: BUS,
    ) -> Result<impl Imu9Dof, DeviceError> {
        match self {
            ImuConfig::Icm20948(config) => {
                let icm = Icm20948::new_spi_from_cfg(spi, *config, Delay);

                // Try to run setup, but with a timeout to detect hanging devices
                let timeout = Duration::from_secs(2);
                icm.initialize_9dof().with_timeout(timeout).await
                    .map_err(|_| DeviceError::Timeout {millis: timeout.as_millis()})?
                    .map_err(icm20948::from_spi_err)
            }
        }
    }

    pub const fn str_id(&self) -> &'static str {
        match self {
            ImuConfig::Icm20948(_) => "ICM-20948",
        }
    }
}
