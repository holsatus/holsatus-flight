use device_driver::{AsyncBufferInterface, AsyncRegisterInterface, RegisterInterfaceBase};
use embassy_time::{Delay, Instant};

use bmi270_driver::{
    Bmi270, Error,
    compat_embedded_hal::{I2cWrap, SpiWrap},
};
use embedded_hal_async::{i2c, spi};
use futures::TryFutureExt as _;

use crate::{
    drivers::{imu::{ImuInitialize, ImuSensor, map_deg_to_rad, map_g_to_mpss}, wrapped::{WrappedI2c, WrappedSpi}}, errors::ImuError, types::measurements::Imu6DofData,
};

impl<I> ImuSensor for Bmi270<I>
where
    ImuError: From<<I as RegisterInterfaceBase>::Error>,
    I: AsyncRegisterInterface<AddressType = u8>,
    I: AsyncBufferInterface<AddressType = u8, Error = <I as RegisterInterfaceBase>::Error>,
{
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.read_acc_scaled()
            .map_ok(map_g_to_mpss)
            .map_err(|e| e.into())
    }

    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.read_gyr_scaled()
            .map_ok(map_deg_to_rad)
            .map_err(|e| e.into())
    }

    fn read_acc_gyr(&mut self) -> impl Future<Output = Result<Imu6DofData<f32>, ImuError>> {
        self.read_acc_gyr_scaled()
            .map_ok(|(acc, gyr)| Imu6DofData {
                timestamp_us: Instant::now().as_micros(),
                gyr,
                acc,
            })
            .map_err(|e| e.into())
    }
}

fn map_spi_error<E: spi::Error>(error: Error<E>, whoami: u8) -> ImuError {
    match error {
        Error::Transport(t) => ImuError::SpiInterface(t.kind().into()),
        Error::ChipId(actual) => ImuError::WhoAmI {
            expected: whoami,
            actual: actual,
        },
        Error::BadInit => ImuError::InitFailure,
        Error::CmdTimeout => ImuError::Timeout,
    }
}

fn map_i2c_error<E: i2c::Error>(error: Error<E>, whoami: u8) -> ImuError {
    match error {
        Error::Transport(t) => ImuError::I2cInterface(t.kind().into()),
        Error::ChipId(actual) => ImuError::WhoAmI {
            expected: whoami,
            actual: actual,
        },
        Error::BadInit => ImuError::InitFailure,
        Error::CmdTimeout => ImuError::Timeout,
    }
}

#[derive(Default)]
pub struct Bmi270Config {
    pub pin_1_int_data_ready: bool,
    pub pin_2_int_data_ready: bool,
}

async fn configure<I: AsyncRegisterInterface<AddressType = u8>>(
    imu: &mut Bmi270<I>,
    config: &Bmi270Config,
) -> Result<(), ImuError>
where
    ImuError: From<I::Error>,
{
    if config.pin_1_int_data_ready {
        imu.map_int1(false, false, true, false).await?;
        imu.configure_int1(true, false, true, false).await?;
    }

    if config.pin_2_int_data_ready {
        imu.map_int2(false, false, true, false).await?;
        imu.configure_int2(true, false, true, false).await?;
    }

    Ok(())
}

pub struct Bmi270Spi;

impl<SPI> ImuInitialize for (Bmi270Spi, SPI)
where
    SPI: spi::SpiDevice,
{
    type Config = Bmi270Config;
    type Interface = SPI;
    type Sensor<'a>
        = Bmi270<SpiWrap<WrappedSpi<&'a mut SPI>>>
    where
        Self: 'a;

    async fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> Result<Self::Sensor<'a>, ImuError>
    where
        Self: 'a,
    {
        let mut imu = Bmi270::initialize_spi(WrappedSpi(interface), Delay)
            .map_err(|error| map_spi_error(error, bmi270_driver::BMI270_CHIP_ID))
            .await?;

        configure(&mut imu, config).await?;

        Ok(imu)
    }
}

pub struct Bmi270I2c;

impl<I2C> ImuInitialize for (Bmi270I2c, I2C)
where
    I2C: i2c::I2c,
{
    type Config = (u8, Bmi270Config);
    type Interface = I2C;
    type Sensor<'a>
        = Bmi270<I2cWrap<WrappedI2c<&'a mut I2C>>>
    where
        Self: 'a;

    async fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> Result<Self::Sensor<'a>, ImuError>
    where
        Self: 'a,
    {
        let mut imu = Bmi270::initialize_i2c(WrappedI2c(interface), config.0, Delay)
            .map_err(|error| map_i2c_error(error, bmi270_driver::BMI270_CHIP_ID))
            .await?;

        configure(&mut imu, &config.1).await?;

        Ok(imu)
    }
}
