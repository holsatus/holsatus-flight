use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

use crate::{common::types::AttitudeReference, messaging as msg, t_librarian::{FmtVec3, FmtVec4}};

const TASK_ID: &str = "[BLACKBOX]";

/// The maximum logging rate in Hz, anything above this will be clamped.
/// Betaflight/Cleanflight default to 3500 micros, which is 285 Hz,
/// and recommend not going lower than 2400 micros, which is 416 Hz. 
const MAX_LOGGING_RATE: u16 = 500;

#[derive(Clone)]
pub enum BlackboxCommand {
    EnableLogging,
    DisableLogging,

    /// Set the max logging rate of the blackbox. The rate is clamped above 250 Hz.
    SetLogRate(u16),
}

#[derive(Clone, Copy)]
struct BlackboxState {
    logging_active: bool,
    log_rate: u16,
}

#[embassy_executor::task]
pub async fn black_box() -> ! {
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();
    let mut rcv_attitude_euler = msg::ATTITUDE_EULER.receiver().unwrap();
    let mut rcv_attitude_setpoint = msg::CMD_ATTITUDE_SETPOINT.receiver().unwrap();
    let mut rcv_controller_output = msg::CONTROLLER_OUTPUT.receiver().unwrap();
    let mut rcv_motor_speeds = msg::MOTOR_SPEEDS.receiver().unwrap();

    let rcv_blackbox_cmd = msg::BLACKBOX_COMMAND_QUEUE.receiver();

    msg::BLACKBOX_COMMAND_QUEUE.send(BlackboxCommand::DisableLogging).await;

    let mut state = BlackboxState {
        logging_active: false,
        log_rate: 50,
    };

    let mut ticker = Ticker::every(Duration::from_hz(state.log_rate as u64));
    '_infinite: loop {

        match select(rcv_blackbox_cmd.receive(), ticker.next()).await {

            // Handle incoming commands
            Either::First(command) => {
                match command {
                    BlackboxCommand::EnableLogging => {
                        defmt::info!("{}: Logging enabled", TASK_ID);
                        state.logging_active = true;
                        ticker = Ticker::every(Duration::from_hz(state.log_rate as u64));
                    },
                    BlackboxCommand::DisableLogging => {
                        defmt::info!("{}: Logging disabled", TASK_ID);
                        state.logging_active = false;
                        ticker = Ticker::every(Duration::from_ticks(u64::MAX/2)); // Hack to push tick far into future
                    },
                    BlackboxCommand::SetLogRate(rate) => {
                        if rate > MAX_LOGGING_RATE {
                            state.log_rate = MAX_LOGGING_RATE;
                            defmt::warn!("{}: Setting log rate to (clamped to 500) {}", TASK_ID, state.log_rate);
                        } else {
                            state.log_rate = rate;
                            defmt::info!("{}: Setting log rate to {}", TASK_ID, state.log_rate);
                        }
                        ticker = Ticker::every(Duration::from_hz(state.log_rate as u64));
                    },
                }
            },

            // If we ticked but logging is not active, do nothing
            Either::Second(_) if !state.logging_active => { }

            // If we ticked and logging is active, log data
            Either::Second(_) => {
                let option_data = (||{

                    let imu_data = rcv_imu_data.try_get()?;
                    let sp = match rcv_attitude_setpoint.try_get() {
                        Some(AttitudeReference::Angle(sp) | AttitudeReference::Rate(sp)) => sp,
                        _ => Vector3::zeros(),
                    };
        
                    Some(crate::t_librarian::DataLogEntry {
                        time_ms: Instant::now().as_millis() as u32,
                        acc: FmtVec3(imu_data.acc),
                        gyr: FmtVec3(imu_data.gyr),
                        mag: FmtVec3(Vector3::zeros()),
                        euler: FmtVec3(rcv_attitude_euler.try_get()?),
                        setpoint: FmtVec3(sp),
                        ctrl_output: FmtVec3(rcv_controller_output.try_get()?),
                        motor_speeds: FmtVec4(rcv_motor_speeds.try_get()?.into()),
                    })
                })();
        
                match option_data {
                    Some(data) => {
                        use msg::LIB_COMMAND_QUEUE;
                        use crate::t_librarian::LibrarianCommand;
                        LIB_COMMAND_QUEUE.sender().send(LibrarianCommand::LogSave(data)).await;
                    }
                    None => {
                        defmt::error!("{}: Failed to fetch some data for logging", TASK_ID);
                    }
                }
            },
        }
    }
}


