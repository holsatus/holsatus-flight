use crate::{errors::SensorError, types::measurements::Imu6DofData};

/// Operations available on an initialized accel+gyro (IMU) sensor.
pub trait AccelGyro: Sized {
    /// Read out a 3D sample from the accelerometer, units are in meters/second^2
    fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>>;

    /// Read out a 3D sample from the gyroscope, units are in radians/second
    fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>>;

    /// Read out 3D samples for both the accelerometer and gyroscope
    fn read_acc_gyr(&mut self) -> impl Future<Output = Result<Imu6DofData<f32>, SensorError>>;
}
