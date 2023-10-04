use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{i2c, peripherals::I2C0};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as CSRMutex, mutex::Mutex};
use nalgebra::Vector3;
use static_cell::StaticCell;
use crate::{channels::{self, Sub}, config::definitions::{ImuFeatures, Configuration}};

#[derive(Clone,Debug)]
pub struct Dof6ImuData<T> {
    pub acc : Vector3<T>,
    pub gyr : Vector3<T>,
}

use defmt::*;

static TASK_ID : &str = "[IMU_MASTER]";

// Shared i2c driver for all IMU devices
static SHARED_ASYNC_I2C : StaticCell<Mutex<CSRMutex, DriverImuI2cType>> = StaticCell::new();
pub type DriverImuI2cType = i2c::I2c<'static, I2C0, i2c::Async>;
pub type SharedImuI2cType = I2cDevice<'static,CSRMutex, DriverImuI2cType>;

#[embassy_executor::task]
pub async fn imu_master(
    spawner: Spawner,
    i2c: DriverImuI2cType,
    config: &'static Configuration,
    mut s_do_gyr_cal: channels::DoGyroCalSub,
    p_imu_reading: channels::ImuReadingPub,
    p_imu0_features: channels::Imu0FeaturesPub,
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

    let imu0_config = config.imu0.unwrap_or_default();
    
    info!("{}: Entering main loop",TASK_ID);
    #[cfg(any(feature = "icm20948-async"))]
    loop {

        // Read and transform data
        let mut reading = sub_icm20948.next_message_pure().await;

        // Apply extrinsics configuration
        imu0_config.imu_ext.translate(&mut reading.acc);
        imu0_config.imu_ext.translate(&mut reading.gyr);
        
        // Apply simple calibration if available for each sensor
        if let Some(gyr_cal) = imu0_config.gyr_cal { gyr_cal.apply(&mut reading.gyr) }
        if let Some(acc_cal) = imu0_config.acc_cal { acc_cal.apply(&mut reading.acc) }

        // Publish reading
        p_imu_reading.publish_immediate(reading);
    
        // Do gyroscope calibration routine
        if Some(true) == s_do_gyr_cal.try_next_message_pure() {
            gyr_cal(
                &mut sub_icm20948,
                imu0_config,
                &p_imu0_features,
                1000
            ).await;
        }
    }
}

/// Subroutine for calibrating a single gyroscpe
async fn gyr_cal(
    sub_imu: &mut Sub<Dof6ImuData<f32>,1>,
    mut imu_config: ImuFeatures,
    p_imu_features: &channels::Imu0FeaturesPub,
    num_samples: usize,
) {
    let mut sum: Vector3<f32> = Vector3::zeros();

    // Get 1000 sampels
    info!("{}: Calibrating IMU gyroscope bias, keep still",TASK_ID);
    for _ in 0..num_samples {
        let mut reading = sub_imu.next_message_pure().await;
        imu_config.imu_ext.translate(&mut reading.gyr) ;
        sum += reading.gyr
    }

    // If calibration values do not already exist, use defaults
    let mut gyr_cal = imu_config.gyr_cal.unwrap_or_default();

    // Unscale and save bias calibration
    gyr_cal.offset = sum.unscale(num_samples as f32);
    imu_config.gyr_cal = Some(gyr_cal);

    // Send updated calibration to configuration task
    p_imu_features.publish_immediate(imu_config);
    info!("{}: IMU gyroscope calibration DONE: {}",TASK_ID,Debug2Format(&imu_config.gyr_cal));
}
