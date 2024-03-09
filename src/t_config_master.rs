use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;

use crate::config::Configuration;

static TASK_ID: &str = "[CONFIG_MASTER]";

use crate::messaging as msg;

// We need to get rid of this!
static CONFIG: static_cell::StaticCell<Configuration> = static_cell::StaticCell::new();

#[embassy_executor::task]
pub async fn config_master(
    mut flash: Flash<'static, FLASH, Async, { crate::config::FLASH_AMT }>,
) {

    // Input channels
    let mut rcv_save_config = msg::RC_SAVE_CONFIG.receiver().unwrap();
    let mut rcv_gyr_calibrations = msg::GYR_CALIBRATIONS.receiver().unwrap();
    let mut rcv_acc_calibrations = msg::ACC_CALIBRATIONS.receiver().unwrap();

    // Output channels
    let snd_config_ref = msg::STATIC_CONFIG_REF.sender();

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

    // NOTE -  Overwrites the loaded transmitter map with the default one
    config.tx_map = crate::transmitter::tx_12_profiles::TX12_DEFAULT_MAP;
    crate::messaging::CFG_TRANSMITTER_MAP.sender().send(config.tx_map);

    // Initialize static and distribute reference
    // TODO - This is not how it should be done. The config governor should split up
    // the config and distribute the parts to the tasks that need them.
    let config_ref = CONFIG.init(config);
    snd_config_ref.send(config_ref);

    defmt::info!("{}: Using the config: {}", TASK_ID,defmt::Debug2Format(&config));

    loop {
        
        // Wait for a signal to save the config
        rcv_save_config.changed_and(|x|x == &true).await;

        defmt::info!("{}: Saving the config to flash", TASK_ID);

        // Update gyroscope calibration values
        if let Some(gyr_calibrations) = rcv_gyr_calibrations.try_changed() {
            for (i, gyr_cal) in gyr_calibrations.iter().enumerate() {
                if let Some(imu_cfg) = config.imu_cfg[i].as_mut() {
                    imu_cfg.gyr_cal = *gyr_cal;
                }
            }
        }

        // Update accelerometer calibration values
        if let Some(acc_calibrations) = rcv_acc_calibrations.try_changed() {
            for (i, acc_cal) in acc_calibrations.iter().enumerate() {
                if let Some(imu_cfg) = config.imu_cfg[i].as_mut() {
                    imu_cfg.acc_cal = *acc_cal;
                }
            }
        }

        defmt::info!("{}: Saving the config: {}", TASK_ID,defmt::Debug2Format(&config));

        // Write inactive config to flash storage if something changed
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