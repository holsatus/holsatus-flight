use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;

use crate::config::Configuration;

static TASK_ID: &str = "[CONFIG_MASTER]";

use crate::messaging as msg;

#[embassy_executor::task]
pub async fn config_master(
    mut flash: Flash<'static, FLASH, Async, { crate::config::FLASH_AMT }>,
) {

    // Input channels
    let mut rcv_save_config = msg::CMD_SAVE_CONFIG.receiver().unwrap();
    let mut rcv_imu_config = msg::CFG_IMU_CONFIG.receiver().unwrap();
    let mut rcv_mag_config = msg::CFG_MAG_CONFIG.receiver().unwrap();
    let mut rcv_attitude_pids = msg::CFG_ATTITUDE_PIDS.receiver().unwrap();
    let mut rcv_transmitter_map = msg::CFG_TRANSMITTER_MAP.receiver().unwrap();

    // Try to load from memory, or use default
    let mut config = match load_config(&mut flash) {
        Ok(config) => {
            defmt::info!("{}: Loaded config from flash", TASK_ID);
            config
        },
        Err(_) => {
            defmt::warn!("{}: Unable to load config from flash. Using default config.", TASK_ID);
            crate::config::DEFAULT_CONFIG
        }
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
    msg::CFG_ATTITUDE_PIDS.sender().send(config.pids);

    // Distribute initial imu (attitude control loop) frequency
    msg::CFG_LOOP_FREQUENCY.sender().send(config.imu_freq);

    // Distribute initial magnetometer frequency
    msg::CFG_MAG_FREQUENCY.sender().send(config.mag_freq);

    // Distribute initial DSHOT speed
    msg::CFG_DSHOT_SPEED.sender().send(config.dshot_speed);

    // Distribute initial DSHOT timeout
    msg::CFG_DSHOT_TIMEOUT.sender().send(config.dshot_timeout);

    // Distribute initial DSHOT timeout
    msg::CFG_RADIO_TIMEOUT.sender().send(config.radio_timeout);

    // Distribute initial DSHOT timeout
    msg::CFG_REVERSE_MOTOR.sender().send(config.motor_dir);

    defmt::info!("{}: Using the config: {}", TASK_ID,defmt::Debug2Format(&config));

    loop {

        // Wait for command to save the config
        rcv_save_config.changed_and(|x|x == &true).await;

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
            config.pids = mew_attitude_pids;
        }

        // Fetch updated transmitter map
        if let Some(new_transmitter_map) = rcv_transmitter_map.try_changed() {
            config.tx_map = new_transmitter_map;
        }

        // Save the config to flash // TODO Consider sequential storage key-value type
        defmt::info!("{}: Saving the config: {}", TASK_ID,defmt::Debug2Format(&config));
        unsafe { crate::config::storage::write(&mut flash, &config, crate::config::CFG_ADDR_OFFSET) };
    }
}

pub fn load_config(
    flash: &mut Flash<'static, FLASH, Async, { crate::config::FLASH_AMT }>,
) -> Result<Configuration,()> {
    use crate::config::{DEFAULT_CONFIG, CFG_ADDR_OFFSET, storage::read};
    let config: Configuration = unsafe { read(flash, CFG_ADDR_OFFSET) };
    if config.header == DEFAULT_CONFIG.header && config.footer == DEFAULT_CONFIG.footer {
        Ok(config)
    } else {
        Err(())
    }
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