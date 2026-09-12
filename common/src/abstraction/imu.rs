use crate::{errors::SensorError, types::measurements::Imu6DofData};

/// A sensor which knows how to initialize itself and use a specific interface kind.
pub trait ImuInitialize {
    type Config;
    type Interface;
    type Sensor<'a>: Imu
    where
        Self: 'a;

    /// Try to initialize the sensor using a borrowed interface and configuration.
    fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> impl Future<Output = Result<Self::Sensor<'a>, SensorError>>
    where
        Self: 'a;
}

/// Operations available on an initialized accel+gyro (IMU) sensor.
pub trait Imu: Sized {
    /// Read out a 3D sample from the accelerometer, units are in meters/second^2
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>>;

    /// Read out a 3D sample from the gyroscope, units are in radians/second
    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>>;

    /// Read out 3D samples for both the accelerometer and gyroscope
    fn read_acc_gyr(&mut self) -> impl Future<Output = Result<Imu6DofData<f32>, SensorError>>;
}
