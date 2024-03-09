use core::{future::poll_fn, pin::pin, task::Poll};

use defmt::{info, warn};
use futures::FutureExt;

use crate::{messaging as msg, sensors::{SensorCondition, SensorRedundancy}};

#[embassy_executor::task]
pub async fn imu_governor() -> ! {
    // Input messages
    let mut rcv_imu_sensor_array: [_; crate::N_IMU] = core::array::from_fn(|i| msg::IMU_SENSOR[i].receiver().unwrap());
    let mut rcv_gyr_calib = msg::GYR_CALIBRATIONS.receiver().unwrap();
    let mut rcv_acc_calib = msg::ACC_CALIBRATIONS.receiver().unwrap();

    // Output messages
    let snd_imu_data = msg::IMU_DATA.sender();
    let snd_imu_active_id = msg::IMU_ACTIVE_ID.sender();
    let snd_gyr_calibrated = msg::GYR_CALIBRATED.sender();
    let snd_acc_calibrated = msg::ACC_CALIBRATED.sender();

    // Setup sensor redundancy manager
    let mut imu_redundancy = SensorRedundancy::<4>::default();

    snd_imu_active_id.send(imu_redundancy.active_id());

    // Receive initial calibration data
    let mut gyr_calib = rcv_gyr_calib.changed().await;
    let mut acc_calib = [None; crate::N_IMU]; // rcv_acc_calib.changed().await;

    'infinite: loop {

        // Check if there is new gyroscope calibration data
        if let Some(new_gyr_calib) = rcv_gyr_calib.try_get() {
            gyr_calib = new_gyr_calib
        }

        // Check if there is new accelerometer calibration data
        if let Some(new_acc_calib) = rcv_acc_calib.try_get() {
            acc_calib = new_acc_calib
        }

        // Wait for data from any IMU sensor by polling each sensor channel
        // This is not beautiful, but it is a decent way to poll an array of channels
        let ((mut data, time), id) = poll_fn(|cx| {
            for (id, imu_sensor) in rcv_imu_sensor_array.iter_mut().enumerate() {
                if let Poll::Ready(data) = pin!(imu_sensor.changed()).poll_unpin(cx) {
                    return Poll::Ready((data, id as u8));
                }
            }
            Poll::Pending
        }).await;

        // Save the timestamp of the reading
        imu_redundancy.get_mut(id).last_reading = Some(time);

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
            warn!("[IMU REDUNDANCY]: IMU sensor {} is degraded", id);
            if imu_redundancy.is_active(id) {
                imu_redundancy.lower_state()
            }
            continue 'infinite;
        }

        // TODO : Add more checks for other sensor failures

        // Transmit state of gyroscope calibration
        match gyr_calib.get(id as usize) {
            Some(Some(_)) if imu_redundancy.is_active(id) => snd_gyr_calibrated.send(true),
            _ => snd_gyr_calibrated.send(false),
        }

        // Transmit state of accelerometer calibration
        match acc_calib.get(id as usize) {
            Some(Some(_)) if imu_redundancy.is_active(id) => snd_acc_calibrated.send(true),
            _ => snd_acc_calibrated.send(false),
        }

        // Send data of active sensor
        if let Some(Some(gyr_cal)) = gyr_calib.get(id as usize) && let Some(Some(acc_cal)) = acc_calib.get(id as usize) && imu_redundancy.is_active(id) {
            gyr_cal.apply(&mut data.gyr);
            acc_cal.apply(&mut data.acc);
            snd_imu_data.send(data)
        }

        // If active sensor changed, send new active sensor id
        if Some(imu_redundancy.active_id()) != snd_imu_active_id.try_get() {
            info!("[IMU REDUNDANCY]: Active IMU sensor changed to {}", imu_redundancy.active_id());
            snd_imu_active_id.send(imu_redundancy.active_id())
        }
    }
}
