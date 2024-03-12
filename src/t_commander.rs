use core::f32::consts::PI;

use embassy_futures::select;
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

use crate::common::types::AttitudeReference;
use crate::constants::PI_D2;
use crate::transmitter::EventRequest;
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
    let snd_start_gyr_calib = msg::CMD_START_GYR_CALIB.sender();
    let snd_start_acc_calib = msg::CMD_START_ACC_CALIB.sender();
    let snd_start_mag_calib = msg::CMD_START_MAG_CALIB.sender();
    let snd_save_config = msg::CMD_SAVE_CONFIG.sender();
    let snd_en_integral = msg::CMD_EN_INTEGRAL.sender();

    enum StabMode {
        Angle,
        Rate,
    }

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
                    EventRequest::Unbound => defmt::error!("{}: Unbound commands should not end up in the request queue!", TASK_ID),
                    EventRequest::ArmMotors => snd_arming_command.send(true),
                    EventRequest::DisarmMotors => snd_arming_command.send(false),
                    EventRequest::AngleMode => stab_mode = StabMode::Angle,
                    EventRequest::RateMode => stab_mode = StabMode::Rate,
                    EventRequest::StartGyrCalib => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_start_gyr_calib.send(true)
                        } else {
                            defmt::warn!("{}: Gyroscope calibration can only happen when disarmed", TASK_ID);
                        }
                    },
                    EventRequest::AbortGyrCalib => snd_start_gyr_calib.send(false),
                    EventRequest::StartAccCalib => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_start_acc_calib.send(true)
                        } else {
                            defmt::warn!("{}: Accelerometer calibration can only happen when disarmed", TASK_ID);
                        }
                    },
                    EventRequest::AbortAccCalib => snd_start_acc_calib.send(false),
                    EventRequest::StartMagCalib => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_start_mag_calib.send(true)
                        } else {
                            defmt::warn!("{}: Magnetometer calibration can only happen when disarmed", TASK_ID);
                        }
                    },
                    EventRequest::AbortMagCalib => snd_start_mag_calib.send(false),
                    EventRequest::SaveConfig => {
                        if rcv_motor_state.get().await.is_disarmed() {
                            snd_save_config.send(true)
                        } else {
                            defmt::warn!("{}: Configuration can only be saved when disarmed", TASK_ID);
                        }
                    },
                    EventRequest::RcFailsafe => snd_arming_command.send(false), // Current failsafe is to disarm
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
                    match landed_state {
                        L::Undefined | L::OnGround => snd_en_integral.send_if_different(false),
                        L::InAir | L::Landing | L::Takeoff => snd_en_integral.send_if_different(true),
                    }
                }
            }
        }
    }
}
