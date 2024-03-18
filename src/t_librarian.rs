use defmt::{write, Format};
use embassy_futures::yield_now;
use nalgebra::{Vector3, Vector4};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{fetch_item, store_item};
use sequential_storage::queue::push;
use crate::bsp::PAGE_SIZE;

use crate::config::{keyed_item::KeyedItem, CFG_MAP_RANGE, CFG_QUEUE_RANGE};

static TASK_ID: &str = "[LIBRARIAN]";

use crate::messaging as msg;

#[allow(unused)]
enum LogEntryType {
    DataLog(DataLogEntry),
    VehicleState(VehicleState),
}

#[allow(unused)]
#[derive(Clone, Format)]
pub struct DataLogEntry {
    pub time_ms: u32,
    pub acc: FmtVec3<f32>,
    pub gyr: FmtVec3<f32>,
    pub mag: FmtVec3<f32>,
    pub euler: FmtVec3<f32>,
    pub setpoint: FmtVec3<f32>,
    pub ctrl_output: FmtVec3<f32>,
    pub motor_speeds: FmtVec4<u16>,
}

/// Dumb wrapper type to allow formatting of nalgebra vectors
#[derive(Clone, Copy)]
pub struct FmtVec4<T>(pub Vector4<T>);

impl <T: Format> Format for FmtVec4<T> {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "[{}, {}, {}, {}]", self.0[0], self.0[1], self.0[2], self.0[3])
    }
}

/// Dumb wrapper type to allow formatting of nalgebra vectors
#[derive(Clone, Copy)]
pub struct FmtVec3<T>(pub Vector3<T>);

impl <T: Format> Format for FmtVec3<T> {
    fn format(&self, fmt: defmt::Formatter) {
        write!(fmt, "[{}, {}, {}]", self.0[0], self.0[1], self.0[2])
    }
}

bitflags::bitflags! {
    struct VehicleState: u8 {
        const ARMED = 0b00000001;
        const IN_AIR = 0b00000010;
        const LANDING = 0b00000100;
        const EMERGENCY = 0b00001000;
        const LOW_BATTERY = 0b00010000;
        const MANUAL_CONTROL = 0b00100000;
        const AUTOPILOT_ENABLED = 0b01000000;
        const GPS_FIX = 0b10000000;
    }
}

impl Default for VehicleState {
    fn default() -> Self {
        VehicleState::IN_AIR | VehicleState::AUTOPILOT_ENABLED
    }
}

#[allow(unused)]
#[derive(Clone)]
pub enum LibrarianCommand {
    LogLoad,
    LogSave(DataLogEntry),
    CfgLoad,
    CfgSave,//(Configuration),
    EraseLog,
    CfgErase,
}

#[embassy_executor::task]
pub async fn librarian(
    mut flash: crate::bsp::FlashPeripheral,
) {

    // Input channels
    let rcv_command_queue = msg::LIB_COMMAND_QUEUE.receiver();

    const QUEUE_NUM_PAGES: u32 = (CFG_QUEUE_RANGE.end - CFG_QUEUE_RANGE.start) / PAGE_SIZE as u32;
    let mut cache = sequential_storage::cache::PagePointerCache::<{QUEUE_NUM_PAGES as usize}>::new();
    let mut data_buffer = [0u8; 2048];

    let mut keyed_config = match fetch_item(&mut flash, CFG_MAP_RANGE, &mut cache, &mut data_buffer, crate::config::CFG_KEY).await {
        Ok(Some(config)) => {
            defmt::info!("{}: Loaded config from flash", TASK_ID);
            config
        },
        Ok(None) => {
            defmt::warn!("{}: Unable to load config from flash. None state.", TASK_ID);
            KeyedItem::new(crate::config::DEFAULT_CONFIG, crate::config::CFG_KEY)
        }
        Err(e) => {
            defmt::warn!("{}: Unable to load config from flash: {:?}", TASK_ID, defmt::Debug2Format(&e));
            KeyedItem::new(crate::config::DEFAULT_CONFIG, crate::config::CFG_KEY)
        },
    };

    
    // NOTE - Overwrites the loaded transmitter map with the default one
    keyed_config.data.tx_map = crate::transmitter::tx_12_profiles::TX12_8CH_DEFAULT_MAP;
    
    let config = &keyed_config.data;
    
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

    //
    // End of copied from t_config_master.rs
    //

    loop {

        let command = rcv_command_queue.receive().await;

        match command {
            LibrarianCommand::LogLoad => {
                defmt::info!("\n\n\n\n\n{}: Log entry loaded from flash", TASK_ID);
                match sequential_storage::queue::peek_many(&mut flash, CFG_QUEUE_RANGE, &mut cache).await {
                    Ok(mut iter) => {
                        while let Ok(Some(buffer)) = iter.next(&mut data_buffer).await {

                            // TURN IT INTO ORIGINAL TYPE WITH TRANSMUTATION
                            // Ensure buffer is large enough to contain the data
                            if buffer.len() < core::mem::size_of::<DataLogEntry>() {
                                continue;
                            }

                            // Relevant slice of original buffer
                            let smaller_buffer = &buffer[..core::mem::size_of::<DataLogEntry>()];

                            // Create a new buffer to store the data
                            let mut datastore = [0u8; core::mem::size_of::<DataLogEntry>()];

                            // Copy the data from the smaller buffer into the datastore
                            for (i, byte) in smaller_buffer.iter().enumerate() {
                                datastore[i] = *byte;
                            }

                            // "Deserialize" the data via transmutation
                            let data: DataLogEntry = unsafe { core::mem::transmute(datastore) };

                            defmt::println!("{}", data);
                            yield_now().await;
                        }
                    },
                    Err(_) => defmt::warn!("{}: Log entry failed to save to flash", TASK_ID),
                }
            },
            LibrarianCommand::LogSave(log) => {
                // defmt::println!("{}", log);
                let slice =  unsafe { core::slice::from_raw_parts((&log as *const DataLogEntry) as *const u8, ::core::mem::size_of::<DataLogEntry>()) };
                match push(&mut flash, CFG_QUEUE_RANGE, &mut cache, slice, true).await {
                    Ok(_) => defmt::info!("{}: Log entry saved to flash", TASK_ID),
                    Err(_) => defmt::warn!("{}: Log entry failed to save to flash", TASK_ID),
                }
            },
            LibrarianCommand::CfgLoad => {
                match fetch_item::<crate::config::Cfg,_>
                (&mut flash, CFG_MAP_RANGE, &mut cache, &mut data_buffer, keyed_config.key()).await {
                    Ok(Some(new_keyed_config)) => {
                        // TODO Do something with the config
                        defmt::info!("{}: Config loaded from flash", TASK_ID);
                        defmt::println!("{}", defmt::Debug2Format(&new_keyed_config.data));
                        keyed_config = new_keyed_config;
                    },
                    Ok(None) => defmt::warn!("{}: Config was not found in flash", TASK_ID),
                    Err(_) => defmt::warn!("{}: Config failed to load from flash", TASK_ID),
                }
            },
            LibrarianCommand::CfgSave => {

                if let Some(new_imu_config) = msg::CFG_IMU_CONFIG.try_get() {
                    keyed_config.data.imu_cfg = new_imu_config;
                }

                // Fetch updated magnetometer configuration values
                if let Some(new_mag_config) = msg::CFG_MAG_CONFIG.try_get() {
                    keyed_config.data.mag_cfg = new_mag_config;
                }

                // Fetch updated attitude PID values
                if let Some(mew_attitude_pids) = msg::CFG_ATTITUDE_PIDS.try_get() {
                    keyed_config.data.attpids = mew_attitude_pids;
                }

                // Fetch updated transmitter map
                if let Some(new_transmitter_map) = msg::CFG_TRANSMITTER_MAP.try_get() {
                    keyed_config.data.tx_map = new_transmitter_map;
                }

                match store_item(&mut flash, CFG_MAP_RANGE, &mut NoCache::new(), &mut data_buffer, &keyed_config).await {
                    Ok(_) => {
                        defmt::info!("{}: Config saved to flash", TASK_ID);
                        defmt::println!("{}", defmt::Debug2Format(&keyed_config.data));
                    },
                    Err(_) => defmt::warn!("{}: Config failed to save to flash", TASK_ID),
                }
            
            },
            LibrarianCommand::EraseLog => {
                match flash.blocking_erase(CFG_QUEUE_RANGE.start, CFG_QUEUE_RANGE.end) {
                    Ok(_) => defmt::info!("{}: Log erased from flash ( 0x{:x} -> 0x{:x} )", TASK_ID, CFG_QUEUE_RANGE.start, CFG_QUEUE_RANGE.end),
                    Err(_) => defmt::warn!("{}: Log failed to erase from flash", TASK_ID),
                }
            },
            LibrarianCommand::CfgErase => {
                match flash.blocking_erase(CFG_MAP_RANGE.start, CFG_MAP_RANGE.end) {
                    Ok(_) => defmt::info!("{}: Config erased from flash ( 0x{:x} -> 0x{:x} )", TASK_ID, CFG_MAP_RANGE.start, CFG_MAP_RANGE.end),
                    Err(_) => defmt::warn!("{}: Config failed to erase from flash", TASK_ID),
                }
            },
        }
    }
}
