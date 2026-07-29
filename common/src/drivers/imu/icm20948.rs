use core::future::Future;
use embassy_time::{Delay, Instant};
use embedded_hal_async::{i2c, spi};
use futures::TryFutureExt as _;
use icm20948_async::{
    I2cDevice, Icm20948, IcmBuilder, MagDisabled, SetupError, SpiDevice, Transport,
};

pub use icm20948_async::{AccDlp, AccRange, AccUnit, Config, GyrDlp, GyrRange, GyrUnit};

use crate::{
    drivers::{imu::{ImuInitialize, ImuSensor}, wrapped::{WrappedI2c, WrappedSpi}}, errors::ImuError, types::measurements::Imu6DofData,
};

pub struct Icm20948Sensor<TRANSPORT> {
    sensor: Icm20948<TRANSPORT, MagDisabled>,
}

impl<TRANSPORT> Icm20948Sensor<TRANSPORT> {
    pub fn new(inner: Icm20948<TRANSPORT, MagDisabled>) -> Self {
        Self { sensor: inner }
    }
}

impl<BUS: Transport> ImuSensor for Icm20948Sensor<BUS>
where
    ImuError: From<BUS::Error>,
{
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.sensor.read_acc().map_err(ImuError::from)
    }

    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.sensor.read_gyr().map_err(ImuError::from)
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, ImuError> {
        let raw = self.sensor.read_6dof().await.map_err(ImuError::from)?;
        Ok(Imu6DofData {
            timestamp_us: Instant::now().as_micros(),
            gyr: raw.gyr,
            acc: raw.acc,
        })
    }
}

// --- Error mapping

fn map_setup_err_i2c<E: i2c::Error>(error: SetupError<E>) -> ImuError {
    match error {
        SetupError::Transport(e) => ImuError::I2cInterface(e.into()),
        SetupError::ImuWhoAmI(actual) => ImuError::WhoAmI {
            expected: 0xEA,
            actual,
        },
        SetupError::MagWhoAmI(actual) => ImuError::WhoAmI {
            expected: 0x09,
            actual,
        },
    }
}

fn map_setup_err_spi<E: spi::Error>(error: SetupError<E>) -> ImuError {
    match error {
        SetupError::Transport(e) => ImuError::SpiInterface(e.into()),
        SetupError::ImuWhoAmI(actual) => ImuError::WhoAmI {
            expected: 0xEA,
            actual,
        },
        SetupError::MagWhoAmI(actual) => ImuError::WhoAmI {
            expected: 0x09,
            actual,
        },
    }
}

// --- Initialization

pub struct Icm209486DofI2c;

impl<BUS> ImuInitialize for (Icm209486DofI2c, BUS)
where
    BUS: i2c::I2c,
{
    type Config = (u8, Config);
    type Interface = BUS;
    type Sensor<'a>
        = Icm20948Sensor<I2cDevice<WrappedI2c<&'a mut BUS>>>
    where
        Self: 'a;

    fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> impl Future<Output = Result<Self::Sensor<'a>, ImuError>>
    where
        Self: 'a,
    {
        // The units are non-negotiable
        let effective_config = Config {
            acc_unit: AccUnit::Mpss,
            gyr_unit: GyrUnit::Rps,
            ..config.1
        };
        IcmBuilder::new_i2c(WrappedI2c(interface), Delay)
            .with_config(effective_config)
            .set_address(config.0)
            .initialize_6dof()
            .map_err(map_setup_err_i2c)
            .map_ok(Icm20948Sensor::new)
    }
}

pub struct Icm209486DofSpi;

impl<BUS> ImuInitialize for (Icm209486DofSpi, BUS)
where
    BUS: spi::SpiDevice,
{
    type Config = Config;
    type Interface = BUS;
    type Sensor<'a>
        = Icm20948Sensor<SpiDevice<WrappedSpi<&'a mut BUS>>>
    where
        Self: 'a;

    fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> impl Future<Output = Result<Self::Sensor<'a>, ImuError>>
    where
        Self: 'a,
    {
        // The units are non-negotiable
        let effective_config = Config {
            acc_unit: AccUnit::Mpss,
            gyr_unit: GyrUnit::Rps,
            ..*config
        };

        IcmBuilder::new_spi(WrappedSpi(interface), Delay)
            .with_config(effective_config)
            .initialize_6dof()
            .map_err(map_setup_err_spi)
            .map_ok(Icm20948Sensor::new)
    }
}
