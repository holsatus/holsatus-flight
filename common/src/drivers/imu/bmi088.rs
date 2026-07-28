use device_driver::AsyncRegisterInterface;
use embassy_time::{Delay, Instant};

use bmi088_driver::{
    Bmi088Accelerometer, Bmi088Gyroscope, Error,
    compat_embedded_hal::{I2cAny, SpiAcc, SpiGyr},
};
use embedded_hal_async::{i2c, spi};
use futures::TryFutureExt as _;

use crate::{
    drivers::imu::{ImuInitialize, ImuSensor, map_deg_to_rad, map_g_to_mpss},
    errors::ImuError,
    types::measurements::Imu6DofData,
};

pub struct Bmi088<IACC, IGYR> {
    acc: Bmi088Accelerometer<IACC>,
    gyr: Bmi088Gyroscope<IGYR>,
}

impl<A: AsyncRegisterInterface<AddressType = u8>, G: AsyncRegisterInterface<AddressType = u8>>
    ImuSensor for Bmi088<A, G>
where
    ImuError: From<A::Error>,
    ImuError: From<G::Error>,
{
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.acc
            .read_acc_scaled()
            .map_ok(map_g_to_mpss)
            .map_err(|e| e.into())
    }

    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
        self.gyr
            .read_gyro_scaled()
            .map_ok(map_deg_to_rad)
            .map_err(|e| e.into())
    }

    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, ImuError> {
        // The BMI088 cannot do a single contiguous read across both sensors :(
        let acc_data = self.read_acc().await?;
        let gyr_data = self.read_gyr().await?;

        Ok(Imu6DofData {
            timestamp_us: Instant::now().as_micros(),
            gyr: gyr_data,
            acc: acc_data,
        })
    }
}

fn map_spi_error<E: spi::Error>(error: Error<E>, whoami: u8) -> ImuError {
    match error {
        Error::Transport(t) => ImuError::SpiInterface(t.kind().into()),
        Error::ChipId(actual) => ImuError::WhoAmI {
            expected: whoami,
            actual: actual,
        },
    }
}

fn map_i2c_error<E: i2c::Error>(error: Error<E>, whoami: u8) -> ImuError {
    match error {
        Error::Transport(t) => ImuError::I2cInterface(t.kind().into()),
        Error::ChipId(actual) => ImuError::WhoAmI {
            expected: whoami,
            actual: actual,
        },
    }
}

#[derive(Default)]
pub struct Bmi088Config {
    pub pin_3_int_data_ready: bool,
    pub pin_4_int_data_ready: bool,
}

pub struct Bmi088Spi;

impl<A, G> ImuInitialize for (Bmi088Spi, A, G)
where
    A: spi::SpiDevice,
    G: spi::SpiDevice,
    ImuError: From<A::Error>,
    ImuError: From<G::Error>,
{
    type Config = Bmi088Config;
    type Interface = (A, G);
    type Sensor<'a>
        = Bmi088<SpiAcc<&'a mut A>, SpiGyr<&'a mut G>>
    where
        Self: 'a;

    async fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> Result<Self::Sensor<'a>, ImuError>
    where
        Self: 'a,
    {
        let acc = Bmi088Accelerometer::initialize_spi(&mut interface.0, &mut Delay)
            .map_err(|error| map_spi_error(error, bmi088_driver::ACC_CHIP_ID))
            .await?;
        let mut gyr = Bmi088Gyroscope::initialize_spi(&mut interface.1, &mut Delay)
            .map_err(|error| map_spi_error(error, bmi088_driver::GYR_CHIP_ID))
            .await?;

        if config.pin_3_int_data_ready {
            gyr.configure_int3(false, true).await?;
            gyr.data_ready_int3_pin(true).await?;
            gyr.enable_data_ready_interrupt(true).await?;
        }

        if config.pin_4_int_data_ready {
            gyr.configure_int4(false, true).await?;
            gyr.data_ready_int4_pin(true).await?;
            gyr.enable_data_ready_interrupt(true).await?;
        }

        Ok(Bmi088 { acc, gyr })
    }
}

pub struct Bmi088I2c;

impl<A, G> ImuInitialize for (Bmi088I2c, A, G)
where
    A: i2c::I2c,
    G: i2c::I2c,
    ImuError: From<A::Error>,
    ImuError: From<G::Error>,
{
    type Config = (u8, u8, Bmi088Config);
    type Interface = (A, G);
    type Sensor<'a>
        = Bmi088<I2cAny<&'a mut A>, I2cAny<&'a mut G>>
    where
        Self: 'a;

    async fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> Result<Self::Sensor<'a>, ImuError>
    where
        Self: 'a,
    {
        let acc = Bmi088Accelerometer::initialize_i2c(&mut interface.0, config.0, &mut Delay)
            .map_err(|error| map_i2c_error(error, bmi088_driver::ACC_CHIP_ID))
            .await?;
        let mut gyr = Bmi088Gyroscope::initialize_i2c(&mut interface.1, config.1, &mut Delay)
            .map_err(|error| map_i2c_error(error, bmi088_driver::GYR_CHIP_ID))
            .await?;

        if config.2.pin_3_int_data_ready {
            gyr.data_ready_int3_pin(true).await?;
            gyr.enable_data_ready_interrupt(true).await?;
        }

        if config.2.pin_4_int_data_ready {
            gyr.data_ready_int4_pin(true).await?;
            gyr.enable_data_ready_interrupt(true).await?;
        }

        Ok(Bmi088 { acc, gyr })
    }
}
