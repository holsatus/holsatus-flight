use core::mem;

use embassy_futures::select::select;
use embassy_rp::flash::{Async, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

use crate::config::{Configuration, CFG_MAP_RANGE};

static TASK_ID: &str = "[CONFIG_MASTER]";

use crate::messaging as msg;

#[embassy_executor::task]
pub async fn config_master(
    mut flash: Flash<'static, FLASH, Async, { crate::config::FLASH_AMT }>,
) {

    // Input channels
    let mut rcv_save_config = msg::CMD_SAVE_CONFIG.receiver().unwrap();
    let mut rcv_reload_config = msg::CMD_RELOAD_CONFIG.receiver().unwrap();
    let mut rcv_imu_config = msg::CFG_IMU_CONFIG.receiver().unwrap();
    let mut rcv_mag_config = msg::CFG_MAG_CONFIG.receiver().unwrap();
    let mut rcv_attitude_pids = msg::CFG_ATTITUDE_PIDS.receiver().unwrap();
    let mut rcv_transmitter_map = msg::CFG_TRANSMITTER_MAP.receiver().unwrap();

    let mut cache = sequential_storage::cache::NoCache::new();
    let mut data_buffer = [0u8; ERASE_SIZE];

    let mut config = match sequential_storage::map::fetch_item::<Configuration,_>
    (&mut flash, CFG_MAP_RANGE, &mut cache, &mut data_buffer, crate::config::CFG_KEY).await {
        Ok(Some(config)) => {
            defmt::info!("{}: Loaded config from flash", TASK_ID);
            config
        },
        Ok(None) => {
            defmt::warn!("{}: Unable to load config from flash. None state.", TASK_ID);
            crate::config::DEFAULT_CONFIG
        }
        Err(e) => {
            defmt::warn!("{}: Unable to load config from flash: {:?}", TASK_ID, defmt::Debug2Format(&e));
            crate::config::DEFAULT_CONFIG
        },
    };

    // NOTE - Overwrites the loaded transmitter map with the default one
    config.tx_map = crate::transmitter::tx_12_profiles::TX12_8CH_DEFAULT_MAP;

    // Distribute initial transmitter map
    msg::CFG_TRANSMITTER_MAP.sender().send(config.tx_map);

    // Distribute initial imu configurations
    msg::CFG_IMU_CONFIG.sender().send(config.imu_cfg);

    // Distribute initial magnetometer configurations
    msg::CFG_MAG_CONFIG.sender().send(config.mag_cfg);

    // Distribute initial motor mixing matrix
    msg::CFG_MIXING_MATRIX.sender().send(config.mixing_matrix);

    // Distribute initial PID values
    msg::CFG_ATTITUDE_PIDS.sender().send(config.attpids);

    // Distribute initial imu (attitude control loop) frequency
    msg::CFG_LOOP_FREQUENCY.sender().send(config.imu_freq);

    // Distribute initial magnetometer frequency
    msg::CFG_MAG_FREQUENCY.sender().send(config.mag_freq);

    // Distribute initial DSHOT speed
    msg::CFG_DSHOT_SPEED.sender().send(config.dshot_speed);

    // Distribute initial DSHOT timeout
    msg::CFG_DSHOT_TIMEOUT.sender().send(config.dshot_timeout);

    // Distribute initial radio timeout
    msg::CFG_RADIO_TIMEOUT.sender().send(config.radio_timeout);

    // Distribute motor spin directions
    msg::CFG_REVERSE_MOTOR.sender().send(config.motor_dir);

    // Distribute default mavlink frequencies
    msg::CFG_MAV_STREAM_FREQ.sender().send(config.mav_freq);

    defmt::info!("{}: Using the config: {}", TASK_ID,defmt::Debug2Format(&config));

    loop {

        // Wait for command to save the config
        match select(
            rcv_save_config.changed_and(|x|x == &true),
            rcv_reload_config.changed_and(|x|x == &true),
        ) .await {
            embassy_futures::select::Either::First(_) => {

                // Fetch updated imu configuration values
                if let Some(new_imu_config) = rcv_imu_config.try_changed() {
                    config.imu_cfg = new_imu_config;
                }

                // Fetch updated magnetometer configuration values
                if let Some(new_mag_config) = rcv_mag_config.try_changed() {
                    config.mag_cfg = new_mag_config;
                }

                // Fetch updated attitude PID values
                if let Some(mew_attitude_pids) = rcv_attitude_pids.try_changed() {
                    config.attpids = mew_attitude_pids;
                }

                // Fetch updated transmitter map
                if let Some(new_transmitter_map) = rcv_transmitter_map.try_changed() {
                    config.tx_map = new_transmitter_map;
                }

                // Save the config to flash // TODO Consider sequential storage key-value type
                defmt::info!("{}: Saving the config: {}", TASK_ID,defmt::Debug2Format(&config));

                let size = mem::size_of::<Configuration>();
                defmt::info!("{}: Size of config: {}", TASK_ID, size);

                match sequential_storage::map::store_item::<Configuration,_>
                (&mut flash, CFG_MAP_RANGE, &mut cache, &mut data_buffer, &config).await {
                    Ok(()) => defmt::info!("{}: Config saved!", TASK_ID),
                    Err(e) => defmt::error!("{}: Error {}", TASK_ID, defmt::Debug2Format(&e)),
                }

            },
            embassy_futures::select::Either::Second(_) => {

                match sequential_storage::map::fetch_item::<Configuration,_>
                (&mut flash, CFG_MAP_RANGE, &mut cache, &mut data_buffer, crate::config::CFG_KEY).await {
                    Ok(Some(new_config)) => {
                        defmt::info!("{}: Loaded config from flash", TASK_ID);
                        config = new_config
                    },
                    Ok(None) => {
                        defmt::warn!("{}: Unable to load config from flash. None state.", TASK_ID);
                    }
                    Err(e) => {
                        defmt::warn!("{}: Unable to load config from flash: {:?}", TASK_ID, defmt::Debug2Format(&e));
                    },
                };

            },
        }

        // unsafe { crate::config::storage::write(&mut flash, &config, crate::config::CFG_ADDR_OFFSET) };
    }
}

pub fn load_config(
    flash: &mut Flash<'static, FLASH, Async, { crate::config::FLASH_AMT }>,
) -> Result<Configuration,()> {
    use crate::config::{CFG_ADDR_OFFSET, storage::read};
    let config: Configuration = unsafe { read(flash, CFG_ADDR_OFFSET) };
    Ok(config)
}

#[derive(Debug, Copy, Clone)]
pub enum ChangeConfig {
    GyrCalib([Option<crate::config::Calibration>; crate::N_IMU]),
    AccCalib([Option<crate::config::Calibration>; crate::N_IMU]),
    MagCalib([Option<crate::config::Calibration>; crate::N_IMU]),
    TxMap(crate::transmitter::TransmitterMap),
    ChangePid(ChangePids),
}

#[derive(Debug, Copy, Clone)]
pub enum ChangePids {
    RollInner(crate::filters::pid_controller::PidConfig<f32>),
    RollOuter(crate::filters::pid_controller::PidConfig<f32>),
    PitchInner(crate::filters::pid_controller::PidConfig<f32>),
    PitchOuter(crate::filters::pid_controller::PidConfig<f32>),
    YawInner(crate::filters::pid_controller::PidConfig<f32>),
    YawOuter(crate::filters::pid_controller::PidConfig<f32>),
}