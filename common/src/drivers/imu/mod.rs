pub mod bmi088;
pub mod bmi270;
pub mod icm20948;

pub use bmi088::{Bmi088Config, Bmi088I2c, Bmi088Spi};
pub use bmi270::{Bmi270Config, Bmi270I2c, Bmi270Spi};
pub use icm20948::{Icm209486DofI2c, Icm209486DofSpi};

use crate::{errors::ImuError, types::measurements::Imu6DofData};

pub fn map_deg_to_rad(arr: [f32; 3]) -> [f32; 3] {
    arr.map(|v| v.to_radians())
}

pub fn map_g_to_mpss(arr: [f32; 3]) -> [f32; 3] {
    arr.map(|v| v * crate::consts::GRAVITY)
}

/// A sensor which knows how to initialize itself and use a specific interface kind.
pub trait ImuInitialize {
    type Config;
    type Interface;
    type Sensor<'a>: ImuSensor
    where
        Self: 'a;

    fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> impl Future<Output = Result<Self::Sensor<'a>, ImuError>>
    where
        Self: 'a;
}

/// Operations available on an initialized IMU sensor.
pub trait ImuSensor: Sized {
    /// Read out a 3D sample from the accelerometer, units are in meters/second^2
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>>;
    /// Read out a 3D sample from the gyroscope, units are in radians/second
    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>>;
    /// Read out 3D samples for both the accelerometer and gyroscope
    fn read_acc_gyr(&mut self) -> impl Future<Output = Result<Imu6DofData<f32>, ImuError>>;
}

pub mod trigger {
    use futures::FutureExt;

    /// Some event trigger to read a sensor. Can be an interrupt, a timer, or an in-software signal.
    pub trait Trigger {
        fn next_trigger(&mut self) -> impl Future<Output = ()>;
    }

    impl Trigger for embassy_time::Ticker {
        fn next_trigger(&mut self) -> impl Future<Output = ()> {
            self.next()
        }
    }

    pub struct OnRising<W>(pub W);
    pub struct OnFalling<W>(pub W);

    impl<W: embedded_hal_async::digital::Wait> Trigger for OnRising<W> {
        fn next_trigger(&mut self) -> impl Future<Output = ()> {
            self.0.wait_for_rising_edge().map(|_| ())
        }
    }

    impl<W: embedded_hal_async::digital::Wait> Trigger for OnFalling<W> {
        fn next_trigger(&mut self) -> impl Future<Output = ()> {
            self.0.wait_for_falling_edge().map(|_| ())
        }
    }
}
