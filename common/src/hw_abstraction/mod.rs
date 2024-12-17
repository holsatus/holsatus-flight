use crate::errors::DeviceError;
use crate::types::measurements::{Imu6DofData, Imu9DofData};

#[allow(async_fn_in_trait)]
pub trait Imu6Dof {
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError>;
    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError>;
    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError>;
}

#[allow(async_fn_in_trait)]
pub trait Imu9Dof: Imu6Dof {
    async fn read_mag(&mut self) -> Result<[f32; 3], DeviceError>;
    async fn read_acc_gyr_mag(&mut self) -> Result<Imu9DofData<f32>, DeviceError>;
}

#[allow(async_fn_in_trait)]
pub trait FourMotors {
    async fn set_motor_speeds(&mut self, speeds: [u16; 4]);
    async fn set_motor_speeds_min(&mut self);
    async fn set_reverse_dir(&mut self, rev: [bool; 4]);
    async fn make_beep(&mut self);
}
