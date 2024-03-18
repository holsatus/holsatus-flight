use core::{future::poll_fn, pin::pin, task::Poll};

use defmt::{info, warn};
use embassy_time::Instant;
use futures::FutureExt;

use crate::{messaging as msg, sensors::{SensorCondition, SensorRedundancy}};

#[embassy_executor::task]
pub async fn imu_governor() -> ! {
    // Input messages
    let mut rcv_imu_sensor_array: [_; crate::N_IMU] = core::array::from_fn(|i| msg::IMU_SENSOR[i].receiver().unwrap());
    let mut rcv_imu_config = msg::CFG_IMU_CONFIG.receiver().unwrap();

    // Output messages
    let snd_imu_data = msg::IMU_DATA.sender();
    let snd_imu_active_id = msg::IMU_ACTIVE_ID.sender();
    let snd_gyr_calibrated = msg::GYR_CALIBRATED.sender();
    let snd_acc_calibrated = msg::ACC_CALIBRATED.sender();

    // Setup sensor redundancy manager
    let mut imu_redundancy = SensorRedundancy::<4>::default();

    snd_imu_active_id.send(imu_redundancy.active_id());

    let mut imu_config = rcv_imu_config.changed().await;

    'infinite: loop {

        // Check if there is new gyroscope calibration data
        if let Some(new_imu_config) = rcv_imu_config.try_get() {
            imu_config = new_imu_config
        }

        // Wait for data from any IMU sensor by polling each sensor channel
        // This is not beautiful, but it is a decent way to poll an array of channels
        let (mut data, id) = poll_fn(|cx| {
            for (id, imu_sensor) in rcv_imu_sensor_array.iter_mut().enumerate() {
                if let Poll::Ready(data) = pin!(imu_sensor.changed()).poll_unpin(cx) {
                    return Poll::Ready((data, id as u8));
                }
            }
            Poll::Pending
        }).await;

        // Save the timestamp of the reading
        imu_redundancy.get_mut(id).last_reading = Some(Instant::now());

        // Early loop-back if sensor is already degraded
        if imu_redundancy.is_degraded(id) {
            continue 'infinite;
        }

        // If sensor condition was previously unknown, set to idle
        if imu_redundancy.get(id).condition == SensorCondition::Unknown {
            imu_redundancy.get_mut(id).condition = SensorCondition::Good
        }

        // Check for sensor stall and mark as degraded if necessary
        imu_redundancy.detect_stall_any();

        // Check if sensor is degraded and lower state if it is active
        if imu_redundancy.is_degraded(id) {
            if imu_redundancy.is_active(id) {
                warn!("[IMU REDUNDANCY]: IMU sensor {} is degraded, attempting fallback.", id);
                imu_redundancy.lower_state();
            }
            continue 'infinite;
        }

        // TODO : Add more checks for other sensor failures

        // If sensor is active, apply calibrations, extrinsics and transmit
        if imu_redundancy.is_active(id) {

            if let Some(Some(config)) = imu_config.get(id as usize)  {
                
                // Apply calibration to gyroscope data
                if let Some(gyr_cal) = config.gyr_cal {
                    gyr_cal.apply(&mut data.gyr);
                }

                // Apply calibration to accelerometer data
                if let Some(acc_cal) = config.acc_cal {
                    acc_cal.apply(&mut data.acc);
                }
                
                // Apply extrinsics to IMU data
                if let Some(imu_ext) = config.imu_ext {
                    imu_ext.apply(&mut data.gyr);
                    imu_ext.apply(&mut data.acc);
                }

                snd_imu_data.send(data)
            }

            // Transmit state of calibrations for current sensor
            if let Some(Some(config)) = imu_config.get(id as usize) {
                snd_gyr_calibrated.send(config.gyr_cal.is_some());
                snd_acc_calibrated.send(config.acc_cal.is_some());
            }
        }

        // If active sensor changed, send new active sensor id
        if Some(imu_redundancy.active_id()) != snd_imu_active_id.try_get() {
            info!("[IMU REDUNDANCY]: Active IMU sensor changed to {}", imu_redundancy.active_id());
            snd_imu_active_id.send(imu_redundancy.active_id())
        }
    }
}
