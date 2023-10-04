pub mod storage;
pub mod definitions;

use defmt;
use embassy_rp::{flash::{Flash, Async}, peripherals::FLASH};
use futures::select_biased;
use futures::FutureExt;

use crate::config::definitions::DEFAULT_CONFIG;

use self::definitions::Configuration;

static TASK_ID : &str = "[CONFIG_MASTER]";

#[embassy_executor::task]
pub async fn config_master(
    mut flash: Flash::<'static, FLASH, Async, { definitions::FLASH_AMT }> ,
    mut s_imu0_features: crate::channels::Imu0FeaturesSub,
    mut s_mag0_features: crate::channels::Mag0FeaturesSub,
) {

    // First load from memory
    let mut config = load_config(&mut flash);

    defmt::info!("{}: Using the config: {}",TASK_ID,defmt::Debug2Format(&config));

    defmt::info!("{}: Entering main loop",TASK_ID);
    loop {

        // Wait for any incoming channels to change
        select_biased! {
            imu_features = s_imu0_features.next_message_pure().fuse() => config.imu0 = Some(imu_features),
            mag_features = s_mag0_features.next_message_pure().fuse() => config.mag0 = Some(mag_features),
        }

        // Check all channels for updates
        if let Some(imu_features) = s_imu0_features.try_next_message_pure() { config.imu0 = Some(imu_features) }
        if let Some(mag_features) = s_mag0_features.try_next_message_pure() { config.mag0 = Some(mag_features) }

        // Write inactive config to flash storage if something changed
        unsafe { storage::write(&mut flash, &config ,definitions::CFG_ADDR_OFFSET) };

    }
}

pub fn load_config( flash: &mut Flash::<'static, FLASH, Async, { definitions::FLASH_AMT }> ) -> Configuration {
    let mut config:Configuration = unsafe { storage::read(flash, definitions::CFG_ADDR_OFFSET) };
    if config.header != definitions::DEFAULT_CONFIG.header || config.footer != definitions::DEFAULT_CONFIG.footer {
        defmt::warn!("{}: Unable to load config from flash storage. Using defaults.",TASK_ID);
        config = DEFAULT_CONFIG;
    }
    config
}