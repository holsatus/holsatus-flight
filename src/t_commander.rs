use embassy_futures::select;
use nalgebra::Vector3;

use crate::common::types::AttitudeReference;
use crate::transmitter::EventRequest;
use crate::messaging as msg;

const TASK_ID : &str = "[COMMANDER]";

#[embassy_executor::task]
pub async fn commander() {

    // Input messages
    let rcv_request_queue = msg::REQUEST_QUEUE.receiver();
    let mut rcv_request_controls = msg::REQUEST_CONTROLS.receiver().unwrap();
    let mut rcv_motor_state = msg::MOTOR_STATE.receiver().unwrap();

    // Output messages
    let snd_attitude_setpoint = msg::ATTITUDE_SETPOINT.sender();
    let snd_throttle_setpoint = msg::THROTTLE_SETPOINT.sender();
    let snd_arming_command = msg::RC_ARMING_CMD.sender();
    let snd_start_gyr_calib = msg::RC_START_GYR_CALIB.sender();
    let snd_start_acc_calib = msg::RC_START_ACC_CALIB.sender();
    let snd_start_mag_calib = msg::RC_START_MAG_CALIB.sender();
    let snd_save_config = msg::RC_SAVE_CONFIG.sender();

    enum StabMode {
        Angle,
        Rate,
    }

    let mut stab_mode = StabMode::Angle;

    defmt::info!("{}: Starting commander task", TASK_ID);
    '_infinite: loop {

        match select::select(
            rcv_request_queue.receive(),
            rcv_request_controls.changed(),
        ).await {

            // Discrete event from queue
            select::Either::First(event) => {

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
            select::Either::Second(control) => {

                let control_vec = Vector3::new(control.roll, control.pitch, control.yaw);

                let attitude_setpoint = match stab_mode {
                    StabMode::Angle => AttitudeReference::Angle(control_vec),
                    StabMode::Rate => AttitudeReference::Rate(control_vec),
                };

                snd_attitude_setpoint.send(attitude_setpoint);
                snd_throttle_setpoint.send(control.thrust);
            },
        }
    }
}
