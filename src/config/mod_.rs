pub mod storage;
pub mod definitions;

mod mod2;

use defmt::*;
use embassy_rp::{flash::{Flash, Async}, peripherals::FLASH};
use embassy_time::{Timer, Duration, Ticker};

use crate::config::{self, definitions::DEFAULT_CONFIG};

use self::definitions::Configuration;

static TASK_ID : &str = "[CONFIG_MASTER]";

static mut CONFIG0: definitions::Configuration = Configuration::const_default();
static mut CONFIG1: definitions::Configuration = Configuration::const_default();

#[embassy_executor::task]
pub async fn config_master(
    mut flash: Flash::<'static, FLASH, Async, { definitions::FLASH_AMT }> ,
    mut s_imu0_features: crate::channels::Imu0FeaturesSub,
    mut s_mag0_features: crate::channels::Mag0FeaturesSub,
    p_config_ref: crate::channels::ConfigRefPub,
) {

    // First load from memory
    unsafe { CONFIG0 = storage::read(&mut flash, definitions::CFG_ADDR_OFFSET) };
    unsafe { CONFIG1 = storage::read(&mut flash, definitions::CFG_ADDR_OFFSET) };

    // Verify header and footer of loaded config, use defaults if check fails
    if (unsafe { CONFIG0.header } != config::definitions::DEFAULT_CONFIG.header ||
        unsafe { CONFIG1.header } != config::definitions::DEFAULT_CONFIG.header ||
        unsafe { CONFIG0.footer } != config::definitions::DEFAULT_CONFIG.footer ||
        unsafe { CONFIG1.footer } != config::definitions::DEFAULT_CONFIG.footer) {
            warn!("{}: Unable to load config from flash storage. Using defaults.",TASK_ID);
            unsafe { CONFIG0 = DEFAULT_CONFIG } ;
            unsafe { CONFIG1 = DEFAULT_CONFIG } ;
    }

    info!("{}: Using the config: {}",TASK_ID,Debug2Format(unsafe{&CONFIG0}));
    
    let mut active_config = ActiveConfig::Config0;

    let mut ticker = Ticker::every(Duration::from_millis(100));
    info!("{}: Entering main loop",TASK_ID);
    loop {

        // Flip active and inactive configs
        active_config.flip_active();

        // Publish reference to the active config
        p_config_ref.publish_immediate(match active_config {
            ActiveConfig::Config0 => unsafe { &CONFIG0 },
            ActiveConfig::Config1 => unsafe { &CONFIG1 },
        });

        // Wait 80 milliseconds for tasks to stop using old config
        Timer::after(Duration::from_millis(80)).await;

        // Start writing to inactive config
        let inactive_config = match active_config {
            ActiveConfig::Config0 => unsafe { &mut CONFIG1 },
            ActiveConfig::Config1 => unsafe { &mut CONFIG0 },
        };

        // Config is updated from inputs
        let mut config_changed = false;
        if let Some(imu_features) = s_imu0_features.try_next_message_pure() { inactive_config.imu0 = Some(imu_features); config_changed = true; }
        if let Some(mag_features) = s_mag0_features.try_next_message_pure() { inactive_config.mag0 = Some(mag_features); config_changed = true; }

        // Write inactive config to flash storage if something changed
        if config_changed {
            unsafe { storage::write(&mut flash, inactive_config ,definitions::CFG_ADDR_OFFSET) };
        }

        // Wait for ticker
        ticker.next().await;

    }
}

enum ActiveConfig {
    Config0,
    Config1
}

impl ActiveConfig {
    fn flip_active(&mut self) {
        match self {
            ActiveConfig::Config0 => *self = ActiveConfig::Config1,
            ActiveConfig::Config1 => *self = ActiveConfig::Config0,
        }
    }
}
