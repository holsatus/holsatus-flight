use defmt::*;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Instant;
use embassy_time::Ticker;
use icm20948_async::*;
use nalgebra::Vector3;
use crate::channels;
use crate::config;
use crate::channels::Ch;

use crate::imu::Dof6ImuData;

pub static ICM20948_IMU_READING: Ch<Dof6ImuData<f32>,1> = PubSubChannel::new();
pub static ICM20948_MAG_READING: Ch<Vector3<f32>,1> = PubSubChannel::new();

pub static FREQUENCY_SIG: PubSubChannel<CriticalSectionRawMutex,f32,1,1,1> = PubSubChannel::new();

static TASK_ID : &str = "[ICM20948_DRIVER]";

#[embassy_executor::task]
pub async fn imu_reader(
    i2c: crate::imu::SharedImuI2cType,
    p_icm20948_imu_reading: channels::Pub<Dof6ImuData<f32>,1>,
    p_icm20948_mag_reading: channels::Pub<Vector3<f32>,1>,
) {
    info!("{}: Starting driver",TASK_ID);

    // Create and await IMU object
    let imu_configured = Icm20948::new(i2c)
        // Configure accelerometer
        .acc_range(AccelerometerRange::Gs8)
        .acc_dlp(AccelerometerDlp::Hz111)
        .acc_unit(AccelerometerUnit::Mpss)
        // Configure gyroscope
        .gyr_range(GyroscopeRange::Dps1000)
        .gyr_dlp(GyroscopeDlp::Hz196)
        .gyr_unit(GyroscopeUnit::Rps)
        // Final initialization
        .set_address(0x69);

    #[cfg(not(feature = "mag"))]
    let imu_result = imu_configured.initialize_6dof().await;

    #[cfg(feature = "mag")]
    let imu_result = imu_configured.initialize_9dof().await;

    // Unpack IMU result safely and print error if necessary
    let mut imu = match imu_result {
        Ok(imu) => imu,
        Err(error) => {
            match error {
                IcmError::BusError(_)   => error!("{}: Communication bus error",TASK_ID),
                IcmError::ImuSetupError => error!("{}: Error during setup",TASK_ID),
                IcmError::MagSetupError => error!("{}: Error during mag setup",TASK_ID)
            } return;
        }
    };

    let mut prev_time = Instant::now();
    let mut average_frequency = 0.0;
    let freq_publisher = FREQUENCY_SIG.immediate_publisher();

    // Read IMU data at constant sample rate
    let mut ticker = Ticker::every(config::definitions::ATTITUDE_LOOP_TIME_DUR);
    info!("{}: Entering main loop",TASK_ID);
    loop {
        if let Ok(imu_data) = imu.read_all().await {

            let imu_data_converted = Dof6ImuData {
                acc: imu_data.acc,
                gyr: imu_data.gyr,
            };

            p_icm20948_imu_reading.publish_immediate(imu_data_converted);

            // Calculate loop time
            let time_now = Instant::now();
            let time_passed = time_now.duration_since(prev_time);
            average_frequency = 0.999*average_frequency + 0.001*(1e6 / time_passed.as_micros() as f64);
            prev_time = time_now;
            freq_publisher.publish_immediate(average_frequency as f32);

            #[cfg(feature = "mag")] {
                p_icm20948_mag_reading.publish_immediate(imu_data.mag);
            }
        } else {
            warn!("Something went wrong reading the IMU!")
        }

        ticker.next().await;
    }
}
