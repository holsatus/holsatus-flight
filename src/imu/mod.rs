use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{i2c::{self, I2c, Async}, peripherals::I2C1};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as CSRMutex, mutex::Mutex};
use nalgebra::Vector3;
use static_cell::StaticCell;
use crate::{channels, cfg};

#[derive(Clone,Debug)]
pub struct Dof6ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
}

// mod accel_cal_routine;

use defmt::*;

static TASK_ID : &str = "[IMU_MASTER]";

// Shared i2c driver for all IMU devices
static SHARED_ASYNC_I2C : StaticCell<Mutex<CSRMutex, I2c<'_, I2C1, Async>>> = StaticCell::new();
pub type DriverImuI2cType = i2c::I2c<'static, I2C1, i2c::Async>;
pub type SharedImuI2cType = I2cDevice<'static,CSRMutex, DriverImuI2cType>;

#[embassy_executor::task]
pub async fn imu_driver(
    spawner: Spawner,
    i2c: DriverImuI2cType,
    p_imu_reading: channels::ImuReadingPub,
) {

    // Setup shared i2c bus and spawn invidual owned bus types
    let shared_i2c = SHARED_ASYNC_I2C.init(Mutex::new(i2c));
    let icm20948_i2c_bus = I2cDevice::new(shared_i2c);

    #[cfg(feature = "icm20948-async")]
    let mut sub_icm20948 = unwrap!(
        crate::drivers::task_icm20948_driver::ICM20948_IMU_READING.subscriber()
    );
    
    #[cfg(feature = "icm20948-async")]
    spawner.must_spawn(crate::drivers::task_icm20948_driver::imu_reader(
        icm20948_i2c_bus, 
        unwrap!(crate::drivers::task_icm20948_driver::ICM20948_IMU_READING.publisher()),
        unwrap!(crate::drivers::task_icm20948_driver::ICM20948_MAG_READING.publisher())
    ));
    
    info!("{}: Entering main loop",TASK_ID);
    #[cfg(any(feature = "icm20948-async"))]
    loop {

        #[cfg(feature = "icm20948-async")]
        {
            let mut reading = sub_icm20948.next_message_pure().await;
            reading.acc = cfg::IMU_ROTATION * reading.acc;
            reading.gyr = cfg::IMU_ROTATION * reading.gyr;
            p_imu_reading.publish_immediate(reading);
        }
    }
}
