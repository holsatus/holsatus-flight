use crate::config::{Configuration, ImuTypeConf};

use embassy_futures::select::{select, Either};
use embassy_time::{with_timeout, Delay, Duration, Instant, Ticker};
use icm20948_async::{Icm20948, IcmError};

use crate::messaging as msg;

#[allow(unused)]
#[embassy_executor::task]
pub async fn icm_20948_driver(
    i2c: crate::main_core0::I2cAsyncType,
    imu_id: u8,
    mag_id: u8,
    config: &'static Configuration,
) -> ! {

    defmt::trace!("[ICM20948 DRIVER]: Driver task started");

    // Input messages
    let mut rcv_imu_active = msg::IMU_ACTIVE_ID.receiver().unwrap();
    let mut rcv_mag_active = msg::MAG_ACTIVE_ID.receiver().unwrap();

    // Output messages
    let snd_imu_sensor = msg::IMU_SENSOR[imu_id as usize].sender();
    let snd_mag_sensor = msg::MAG_SENSOR[mag_id as usize].sender();

    let Some(ImuTypeConf::Icm20948(icm_config)) = config.imu_cfg[0].unwrap().imu_type else {
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

    let imu_config = config.imu_cfg[imu_id as usize].unwrap();
    let mag_config = config.mag_cfg[mag_id as usize].unwrap();

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
        let mut imu_ticker = Ticker::every(Duration::from_hz(config.imu_freq as u64));
        let mag_duration = Duration::from_hz(config.mag_freq as u64);
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
                    if let Some(imu_ext) = imu_config.imu_ext {
                        imu_ext.apply(&mut imu_data.gyr);
                        imu_ext.apply(&mut imu_data.acc);
                    }
                    if let Some(mag_ext) = mag_config.mag_ext {
                        mag_ext.apply(&mut imu_data.mag);
                    }
                    snd_imu_sensor.send((imu_data.into(), Instant::now()));
                    snd_mag_sensor.send((imu_data.mag, Instant::now()));
                    mag_timer += mag_duration;
                }

            // Read and apply calibration to IMU data only
            } else if imu_is_active {
                if let Ok(mut imu_data) = imu.read_6dof().await {
                    if let Some(imu_ext) = imu_config.imu_ext {
                        imu_ext.apply(&mut imu_data.gyr);
                        imu_ext.apply(&mut imu_data.acc);
                    }
                    snd_imu_sensor.send((imu_data.into(), Instant::now()));
                }

            // Read and apply calibration to MAG data only
            } else if mag_is_active && Instant::now() > mag_timer {
                if let Ok(mut mag_data) = imu.read_mag().await {
                    if let Some(mag_ext) = mag_config.mag_ext {
                        mag_ext.apply(&mut mag_data);
                    }
                    snd_mag_sensor.send((mag_data, Instant::now()));
                    mag_timer += mag_duration;
                }
            }

            // Delay until next loop
            imu_ticker.next().await;
        }
    }
}
