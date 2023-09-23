use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{i2c, peripherals::I2C1};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use nalgebra::Vector3;
use crate::{channels, cfg};

#[cfg(feature = "icm20948-async")]
pub mod task_icm20948_driver;

#[derive(Clone,Debug)]
pub struct Dof6ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
}

use defmt::*;

static TASK_ID : &str = "[IMU-DRIVER]";

#[embassy_executor::task]
pub async fn imu_driver(
    spawner: Spawner,
    i2c: I2cDevice<'static,CriticalSectionRawMutex, i2c::I2c<'static, I2C1, i2c::Async>>,
    pub_imu_raw: channels::ImuReadingPub,
) {
    
    #[cfg(feature = "icm20948-async")]
    let mut sub_icm20948 = unwrap!(task_icm20948_driver::ICM20948_IMU_READING.subscriber());
    
    #[cfg(feature = "icm20948-async")]
    spawner.must_spawn(task_icm20948_driver::imu_reader(
        i2c, 
        unwrap!(task_icm20948_driver::ICM20948_IMU_READING.publisher()),
        unwrap!(task_icm20948_driver::ICM20948_MAG_READING.publisher())
    ));
    
    info!("{} : Entering main loop",TASK_ID);
    loop {
        let mut reading = sub_icm20948.next_message_pure().await;

        reading.acc = cfg::IMU_ROTATION * reading.acc;
        reading.gyr = cfg::IMU_ROTATION * reading.gyr;

        pub_imu_raw.publish_immediate(reading);
    }
}
