use crate::config::ImuTypeConf;

use embassy_futures::select::{select, Either};
use embassy_time::{with_timeout, Delay, Duration, Instant, Ticker};
use icm20948_async::{Icm20948, IcmError};

use crate::messaging as msg;

#[allow(unused)]
#[embassy_executor::task]
pub async fn icm_20948_driver(
    i2c: crate::AsyncI2cDevice,
    imu_id: u8,
    mag_id: u8,
) -> ! {

    // Ensure that the IMU and MAG IDs are within bounds
    defmt::assert!((imu_id as usize) < crate::N_IMU, "[ICM20948 DRIVER]: IMU ID {} out of bounds (max is {})", imu_id, crate::N_IMU - 1);
    defmt::assert!((mag_id as usize) < crate::N_MAG, "[ICM20948 DRIVER]: MAG ID {} out of bounds (max is {})", mag_id, crate::N_MAG - 1);

    defmt::trace!("[ICM20948 DRIVER]: Driver task started");

    // Input messages
    let mut rcv_imu_active = msg::IMU_ACTIVE_ID.receiver().unwrap();
    let mut rcv_mag_active = msg::MAG_ACTIVE_ID.receiver().unwrap();

    // Output messages
    let snd_imu_sensor = msg::IMU_SENSOR[imu_id as usize].sender();
    let snd_mag_sensor = msg::MAG_SENSOR[mag_id as usize].sender();

    // Get the IMU and MAG configurations
    let imu_config = msg::CFG_IMU_CONFIG.spin_get().await[imu_id as usize].unwrap();
    let mag_config = msg::CFG_MAG_CONFIG.spin_get().await[mag_id as usize].unwrap();

    let Some(ImuTypeConf::Icm20948(icm_config)) = imu_config.imu_type else {
        panic!("[ICM20948 DRIVER]: Incorrect config provided")
    };

    let imu_future = Icm20948::new_i2c_from_cfg(i2c, icm_config, Delay)
        .set_address(0x69)
        .initialize_9dof();

    let Ok(imu_configured) = with_timeout(Duration::from_secs(2), imu_future).await else {
        panic!("[ICM20948 DRIVER]: IMU failed to initialize within the timeout period")
    };

    let mut imu = match imu_configured {
        Err(IcmError::BusError(_)) => panic!("[ICM20948 DRIVER]: Error on communication bus"),
        Err(IcmError::ImuSetupError) => panic!("[ICM20948 DRIVER]: Unexpected IMU whoami"),
        Err(IcmError::MagSetupError) => panic!("[ICM20948 DRIVER]: Unexpected MAG whoami"),
        Ok(imu) => imu,
    };

    defmt::trace!("[ICM20948 DRIVER]: Finished setting up");

    let mut imu_is_active: bool = false;
    let mut mag_is_active: bool = false;

    'infinite: loop {
        // Wait on any of the two sensors to become active
        match select(
            rcv_imu_active.changed_and(|id| id == &Some(imu_id)),
            rcv_mag_active.changed_and(|id| id == &Some(mag_id)),
        ).await {
            Either::First(_) => imu_is_active = true,
            Either::Second(_) => mag_is_active = true,
        }

        // Ensure that the IMU is supposed to be active
        if let Some(Some(active_imu_id)) = rcv_imu_active.try_changed() {
            imu_is_active = (active_imu_id == imu_id)
        }

        // Ensure that the MAG is supposed to be active
        if let Some(Some(active_mag_id)) = rcv_mag_active.try_changed() {
            mag_is_active = (active_mag_id == mag_id);
        }

        defmt::info!("[ICM20948 DRIVER]: IMU active: {} - MAG active: {}", imu_is_active, mag_is_active);
        
        // Create time keeping objects // TODO Get the frequency from the governor
        let loop_frequency = msg::CFG_LOOP_FREQUENCY.spin_get().await;
        let mag_frequency = msg::CFG_MAG_FREQUENCY.spin_get().await;
        let mut imu_ticker = Ticker::every(Duration::from_hz(loop_frequency as u64));
        let mag_duration = Duration::from_hz(mag_frequency as u64);
        let mut mag_timer = Instant::now();

        while imu_is_active || mag_is_active {

            // Ensure that the IMU is supposed to be active
            if let Some(Some(active_imu_id)) = rcv_imu_active.try_changed() {
                imu_is_active = (active_imu_id == imu_id)
            }

            // Ensure that the MAG is supposed to be active
            if let Some(Some(active_mag_id)) = rcv_mag_active.try_changed() {
                mag_is_active = (active_mag_id == mag_id);
            }

            // Read and apply calibration to both IMU and MAG data
            if imu_is_active && mag_is_active && Instant::now() > mag_timer {
                if let Ok(mut imu_data) = imu.read_9dof().await {
                    snd_imu_sensor.send(imu_data.into());
                    snd_mag_sensor.send(imu_data.mag);
                    mag_timer += mag_duration;
                }

            // Read and apply calibration to IMU data only
            } else if imu_is_active {
                if let Ok(mut imu_data) = imu.read_6dof().await {
                    snd_imu_sensor.send(imu_data.into());
                }

            // Read and apply calibration to MAG data only
            } else if mag_is_active && Instant::now() > mag_timer {
                if let Ok(mut mag_data) = imu.read_mag().await {
                    snd_mag_sensor.send(mag_data);
                    mag_timer += mag_duration;
                }
            }

            // Delay until next loop
            imu_ticker.next().await;
        }
    }
}
