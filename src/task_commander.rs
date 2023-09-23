use core::f32::consts::PI;

use defmt::*;
use embassy_time::{Timer, Duration};
use nalgebra::Vector3;

use crate::{
    task_attitude_controller::StabilizationMode::{Horizon,Acro, self},
    functions::{map, self},
    sbus_cmd::TriSwitch,
    task_blinker::BlinkerMode,
    channels, task_flight_detector::FlightMode, task_motor_governor::{MotorState, ArmedState, DisarmReason}
};

static TASK_ID : &str = "COMMANDER";

#[embassy_executor::task]
pub async fn commander(
    mut s_sbus_cmd: channels::SbusCmdSub,
    mut s_motor_state : channels::MotorStateSub,
    mut s_attitude_sense : channels::AttitudeSenseSub,
    mut s_flight_mode: channels::FlightModeSub,
    p_motor_arm: channels::MotorArmPub,
    p_motor_dir: channels::MotorDirPub,
    p_thrust_cmd: channels::ThrustActuatePub,
    p_blinker_mode: channels::BlinkerModePub,
    p_attitude_int_reset : channels::AttitudeIntResetPub,
    p_attitude_stab_mode : channels::AttitudeStabModePub,
) {

    p_motor_dir.publish_immediate((true,true,false,false));
    p_blinker_mode.publish_immediate(BlinkerMode::OnOffFast);

    let mut ref_yaw = 0.0;
    let mut current_stab_mode: Option<StabilizationMode> = None;
    let mut current_motor_state = MotorState::Disarmed(DisarmReason::NotInitialized);

    info!("{} : Entering main loop",TASK_ID);
    loop {

        let mut reset_controllers = false;

        // Handle RC inputs
        match s_sbus_cmd.next_message_pure().await {

            // Valid command received, send arming switch
            Ok(cmd) => {

                p_motor_arm.publish_immediate(!cmd.sw_b.is_idle());

                // Modify thrust mode based on switch position 
                let thrust = match cmd.sw_c {
                    TriSwitch::Idle   => map(thrust_curve(cmd.thrust), 0., 1., 400., 1000.),
                    TriSwitch::Middle => map(thrust_curve(cmd.thrust), 0., 1., 250., 1500.),
                    TriSwitch::Active => map(thrust_curve(cmd.thrust), 0., 1., 250., 2047.),
                };

                p_thrust_cmd.publish_immediate( thrust );

                let stab_mode = match cmd.sw_f {
                    TriSwitch::Idle | TriSwitch::Middle => {
                        ref_yaw = functions::wrap(ref_yaw - cmd.yaw*0.01, -PI, PI);
                        Horizon(Vector3::new(
                            roll_curve(cmd.roll),
                            pitch_curve(cmd.pitch),
                            yaw_curve(ref_yaw)
                        ))
                    },
                    TriSwitch::Active => {
                        Acro(Vector3::new(
                            roll_curve(cmd.roll*10.),
                            pitch_curve(cmd.pitch*10.),
                            yaw_curve(-cmd.yaw*5.)
                        ))
                    },
                };

                p_attitude_stab_mode.publish_immediate(stab_mode);

                // Check if stabilization mode has changed or is uninitialized
                if current_stab_mode.is_some_and(|c| !c.same_variant_as(&stab_mode)) || current_stab_mode.is_none() {
                    let mode_str = match stab_mode { Horizon(_) => "Horizon", Acro(_) => "Acro" };
                    info!("{} : Stabilization mode changed -> {}",TASK_ID,mode_str);
                    current_stab_mode = Some(stab_mode);
                    reset_controllers = true;
                }
            },

            // SBUS failsafe, connection to remote control lost
            Err(error) => {
                p_motor_arm.publish_immediate(false);
                reset_controllers = true;
                error!("{} : SBUS error -> {}",TASK_ID,error);
                Timer::after(Duration::from_hz(5)).await;
            },
        }

        // Reset controller if craft just changed flight mode
        if let Some(flight_mode) = s_flight_mode.try_next_message_pure() {
            if flight_mode == FlightMode::Ground {
                reset_controllers = true;
            }
        }                

        // Reset controller integrals if motors was just armed
        if let Some(motor_state) = s_motor_state.try_next_message_pure() {
            if !functions::variant_eq(&current_motor_state, &motor_state) {
                current_motor_state = motor_state;
                reset_controllers = true;
            }
        }

        // Map motor states to LED blinker
        p_blinker_mode.publish_immediate(match current_motor_state {
            MotorState::Armed(ArmedState::Idle)         => BlinkerMode::OnOffFast,
            MotorState::Armed(ArmedState::Running(_))   => BlinkerMode::TwoFast,
            MotorState::Disarmed(_)                     => BlinkerMode::OnOffSlow,
        });

        // Reset integral and yaw angle 
        if reset_controllers {
            p_attitude_int_reset.publish_immediate(true);
            ref_yaw = s_attitude_sense.next_message_pure().await.0.z;
        }
    }
}


fn pitch_curve(input : f32) -> f32 {
    input*0.5
}

fn roll_curve(input : f32) -> f32 {
    input*0.5
}

fn yaw_curve(input : f32) -> f32 {
    input
}

fn thrust_curve(input : f32) -> f32 {
    input
}