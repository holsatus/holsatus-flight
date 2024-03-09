use core::{future::poll_fn, pin::pin, task::Poll};

use embassy_time::Instant;
use futures::FutureExt;

use crate::messaging as msg;

#[embassy_executor::task]
pub async fn mag_governor() {

    // Input messages
    let mut rcv_mag_sensor_array: [_; crate::N_MAG] = core::array::from_fn(|i| msg::MAG_SENSOR[i].receiver().unwrap());

    // Output messages
    let snd_mag_data = msg::MAG_DATA.sender();
    let snd_mag_active_id = msg::MAG_ACTIVE_ID.sender();

    // Setup sensor redundancy manager
    let mut imu_redundancy = crate::sensors::SensorRedundancy::<4>::default();

    'infinite: loop {
        
        // Wait for data from any MAG sensor
        let ((data, _time), id) = poll_fn(|cx| {
            for (id, mag_sensor) in rcv_mag_sensor_array.iter_mut().enumerate() {
                if let Poll::Ready(data) = pin!(mag_sensor.changed()).poll_unpin(cx) {
                    return Poll::Ready((data, id as u8));
                }
            }
            Poll::Pending
        }).await;

        imu_redundancy.get_mut(id).last_reading = Some(Instant::now());

        // Early loop-back if sensor is already degraded
        if imu_redundancy.is_degraded(id) {
            continue 'infinite;
        }

        // If sensor condition was previously unknown, set to idle
        if imu_redundancy.get(id).condition == crate::sensors::SensorCondition::Unknown {
            imu_redundancy.get_mut(id).condition = crate::sensors::SensorCondition::Good
        }

        // Check for sensor stall and mark as degraded if necessary
        imu_redundancy.detect_stall_any();

        // Check if sensor is degraded and lower state if it is active
        if imu_redundancy.is_degraded(id) {
            if imu_redundancy.is_active(id) {
                imu_redundancy.lower_state()
            }
            continue 'infinite;
        }

        // TODO : Add more checks for other sensor failures

        // Send data of active sensor
        if imu_redundancy.is_active(id) {
            snd_mag_data.send(data)
        }

        snd_mag_active_id.send(imu_redundancy.active_id())
    }
}
