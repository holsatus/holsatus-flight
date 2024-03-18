use core::f32::consts::PI;

use embassy_futures::select;
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

use crate::common::types::AttitudeReference;
use crate::constants::PI_D2;
use crate::t_blackbox_logging::BlackboxCommand;
use crate::t_librarian::LibrarianCommand;
use crate::messaging as msg;

const TASK_ID : &str = "[COMMANDER]";

#[embassy_executor::task]
pub async fn commander() {

    defmt::info!("{}: Starting commander task", TASK_ID);

    // Input messages
    let rcv_request_queue = msg::REQUEST_QUEUE.receiver();
    let mut rcv_request_controls = msg::REQUEST_CONTROLS.receiver().unwrap();
    let mut rcv_motor_state = msg::MOTOR_STATE.receiver().unwrap();
    let mut rcv_landed_state = msg::LANDED_STATE.receiver().unwrap();
    let mut rcv_attitude_euler = msg::ATTITUDE_EULER.receiver().unwrap();

    // Output messages
    let snd_attitude_setpoint = msg::CMD_ATTITUDE_SETPOINT.sender();
    let snd_throttle_setpoint = msg::CMD_THROTTLE_SETPOINT.sender();
    let snd_arming_command = msg::CMD_ARM_VEHICLE.sender();
    let snd_start_gyr_calib = msg::CMD_GYR_CALIB.sender();
    let snd_start_acc_calib = msg::CMD_START_ACC_CALIB.sender();
    let snd_start_mag_calib = msg::CMD_START_MAG_CALIB.sender();
    let snd_en_integral = msg::CMD_EN_INTEGRAL.sender();
    let snd_librarian_cmd = msg::LIB_COMMAND_QUEUE.sender();
    let snd_blackbox_cmd = msg::BLACKBOX_COMMAND_QUEUE.sender();

    msg::VEHICLE_STATE.sender().send(crate::common::types::VehicleState::Boot);

    enum StabMode {
        Angle,
        Rate,
    }

    let mut enable_logging = false;

    let mut stab_mode = StabMode::Angle;

    let mut ticker = Ticker::every(Duration::from_hz(20));

    let mut prev_request_controls_time = Instant::now();
    let mut yaw_integrated = rcv_attitude_euler.get().await.z;

    '_infinite: loop {

        // Futures are polled in the order they are defined (importance)
        match select::select3(

            // Handle discrete events first, if any
            rcv_request_queue.receive(),

            // Handle continuous events from controls
            rcv_request_controls.changed(),

            // Periodic checks (runs at 20Hz)
            ticker.next()

        ).await {

            // Discrete event from queue
            select::Either3::First(event) => {

                match event {
                    CommanderRequest::ArmMotors(arm) => snd_arming_command.send(arm),
                    CommanderRequest::AngleMode => stab_mode = StabMode::Angle,
                    CommanderRequest::RateMode => stab_mode = StabMode::Rate,

                    // Actually, I do not like this. I would rather have a separate command for each
                    // This method requires the commander to "look into" the command, which is not ideal
                    CommanderRequest::CalGyrCommand(cal_gyr_command) => {
                        match cal_gyr_command {
                            crate::t_gyr_calibration::GyrCalCommand::Start(_) => {
                                if rcv_motor_state.get().await.is_disarmed() {
                                    defmt::warn!("{}: Starting gyroscope calibration", TASK_ID);
                                    snd_start_gyr_calib.send(cal_gyr_command)
                                } else {
                                    defmt::warn!("{}: Gyroscope calibration can only happen when disarmed", TASK_ID);
                                }
                            },
                            crate::t_gyr_calibration::GyrCalCommand::Stop => {
                                defmt::info!("{}: Aborting gyroscope calibration", TASK_ID);
                                snd_start_gyr_calib.send(cal_gyr_command)
                            },
                        }                        
                    },
                    CommanderRequest::StartAccCalib { sensor: _sensor, duration: _duration, max_var: _max_var } => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_start_acc_calib.send(true)
                        } else {
                            defmt::warn!("{}: Accelerometer calibration can only happen when disarmed", TASK_ID);
                        }
                    },
                    CommanderRequest::AbortAccCalib => snd_start_acc_calib.send(false),
                    CommanderRequest::StartMagCalib { sensor: _sensor, max_var: _max_var } => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_start_mag_calib.send(true)
                        } else {
                            defmt::warn!("{}: Magnetometer calibration can only happen when disarmed", TASK_ID);
                        }
                    },
                    CommanderRequest::AbortMagCalib => snd_start_mag_calib.send(false),
                    CommanderRequest::SaveConfig { config_id: _ } => { // TODO use ID
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_librarian_cmd.send(LibrarianCommand::CfgSave).await;
                        } else {
                            defmt::warn!("{}: Configuration can only be saved when disarmed", TASK_ID);
                        }
                    },
                    CommanderRequest::LoadConfig { config_id: _ } => { // TODO use ID
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_librarian_cmd.send(LibrarianCommand::CfgLoad).await;
                        } else {
                            defmt::warn!("{}: Configuration can only be loaded when disarmed", TASK_ID);
                        }
                    
                    },
                    CommanderRequest::ClearConfig => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_librarian_cmd.send(LibrarianCommand::CfgErase).await;
                        } else {
                            defmt::warn!("{}: Configuration can only be cleared when disarmed", TASK_ID);
                        }
                    },
                    CommanderRequest::ConfigureLogging { frequency, enabled, force_enable } => {
                        if let Some(freq) = frequency {
                            if freq == 0 {
                                defmt::warn!("{}: Logging frequency cannot be zero, disabling", TASK_ID);
                                snd_blackbox_cmd.send(BlackboxCommand::DisableLogging).await;
                                continue;
                            } else {
                                snd_blackbox_cmd.send(BlackboxCommand::SetLogRate(freq)).await;
                            }
                        }

                        match enabled {
                            Some(true) => {
                                defmt::info!("{}: Logging enabled", TASK_ID);
                                snd_blackbox_cmd.send(BlackboxCommand::EnableLogging).await
                            },
                            Some(false) => {
                                defmt::info!("{}: Logging disabled", TASK_ID);
                                snd_blackbox_cmd.send(BlackboxCommand::DisableLogging).await
                            },
                            None => {},
                        }

                        if let Some(force) = force_enable {
                            enable_logging = force;
                        }
                    }
                    CommanderRequest::SaveTxMap { config_id: _config_id } => todo!(),
                    CommanderRequest::LoadTxMap { config_id: _config_id } => todo!(), // Current failsafe is to disarm
                    CommanderRequest::RcFailsafe => snd_arming_command.send(false),
                }
            },

            // Continuous event from controls
            select::Either3::Second(control) => {

                let attitude_setpoint: AttitudeReference;
                match stab_mode {

                    // When in angle mode, we integrate the yaw control to get the desired angle
                    StabMode::Angle => {
                        
                        // If the vehicle is armed, and control command is not stale, integrate the yaw control
                        let time_now = Instant::now();
                        if prev_request_controls_time.elapsed() < Duration::from_millis(100)
                        && rcv_motor_state.get().await.is_armed() {
                            yaw_integrated += control.yaw * (time_now.duration_since(prev_request_controls_time).as_micros() as f32 / 1e6);
                            yaw_integrated = crate::functions::wrap(yaw_integrated, -PI, PI);
                        } else {
                            yaw_integrated = rcv_attitude_euler.get().await.z;

                        }

                        prev_request_controls_time = time_now;

                        // Clamp the roll and pitch, in case of extreme commands
                        let control_roll = control.roll.clamp(-PI_D2, PI_D2);
                        let control_pitch = control.pitch.clamp(-PI_D2, PI_D2);

                        attitude_setpoint = AttitudeReference::Angle(Vector3::new(control_roll, control_pitch, yaw_integrated));
                    },

                    // When in rate mode, we directly use the controls as the desired rate
                    StabMode::Rate => attitude_setpoint = {
                        let rate_vector = Vector3::new(control.roll, control.pitch, control.yaw);
                        
                        // NOTE We double the rate_vector to get closer to a typical rate.
                        // We might want to give this option to the user, but this is probably a good default.
                        AttitudeReference::Rate(rate_vector * 2.0)
                    },
                }

                snd_attitude_setpoint.send(attitude_setpoint);
                snd_throttle_setpoint.send(control.thrust);
            },

            // Periodic checks go here, general business logic
            select::Either3::Third(_) => {

                // Enable or disable attitude control integral term if not airborne
                if let Some(landed_state) = rcv_landed_state.try_changed() {
                    use crate::t_flight_detector::LandedState as L;
                    let airborne = match landed_state {
                        L::Undefined | L::OnGround => false,
                        L::InAir | L::Landing | L::Takeoff => true,
                    };

                    // Send if different from current value
                    if Some(airborne) != snd_en_integral.try_get() {
                        snd_en_integral.send(airborne);
                    }

                    if enable_logging && airborne {
                        snd_blackbox_cmd.send(BlackboxCommand::EnableLogging).await;
                    }
                }
            }
        }
    }
}


#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RequestOrigin {
    RemoteControl,
    Offboard,
    Shell,
}

// TODO Move to a more appropriate location
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CommanderRequest {

    /// Arm or disarm all motors
    ArmMotors(bool),

    /// Change stabilization mode to angle (or horizon) mode
    AngleMode,

    /// Change stabilization mode to rate (or acro / 3D) mode
    RateMode,

    /// Start gyroscope calibration
    CalGyrCommand(crate::t_gyr_calibration::GyrCalCommand),

    /// Start accelerometer calibration
    StartAccCalib {

        // ID of sensor to calibrate, or `None` for all available sensors.
        sensor: Option<u8>,

        /// Duration of calibration in seconds, or `None` for default duration of 5 seconds.
        duration: Option<f32>,

        /// Maximum variance of the accelerometer data during calibration, or `None` for default value of 0.002.
        max_var: Option<f32>,
    },

    /// Abort any currently active accelerometer calibration
    AbortAccCalib,

    /// Start magnetometer calibration
    StartMagCalib {
            
        // ID of sensor to calibrate, or `None` for all available sensors.
        sensor: Option<u8>,

        /// Maximum variance of the spherical deviation for measurements, or `None` for default value of 0.01. // TODO Check this
        max_var: Option<f32>,
    },

    /// Abort any currently active magnetometer calibration
    AbortMagCalib,

    /// Save any currently modified parameters to flash
    SaveConfig {
                
        /// ID of the configuration to save, or `None` for the default configuration (0).
        config_id: Option<u8>,
    },

    /// Save any currently modified parameters to flash
    LoadConfig {
            
        /// ID of the configuration to load, or `None` for the default configuration (0).
        config_id: Option<u8>,
    },

    /// Clear the configuration from flash. This will force-reset all saved parameters.
    ClearConfig,

    /// Configure the blackbox logger
    ConfigureLogging {
            
        /// Frequency of logging in Hz, or `None` for default frequency of 50Hz.
        frequency: Option<u16>,

        /// Enables or disables automatic logging when the vehice is armed.
        enabled: Option<bool>,

        /// Forces the logger to start logging, even if the vehicle is disarmed.
        force_enable: Option<bool>,
    },

    /// Save the current transmitter mapping profile to flash
    SaveTxMap {
                
        /// ID of the configuration to save, or `None` for the default configuration (0).
        config_id: Option<u8>,
    },

    /// Load transmitter mapping profile to flash
    LoadTxMap{
            
        /// ID of the configuration to load, or `None` for the default configuration (0).
        config_id: Option<u8>,
    },

    /// Report a remote-control RC Failsafe
    RcFailsafe,
}
