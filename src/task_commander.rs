use core::f32::consts::PI;

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Timer, Ticker, Instant};
use nalgebra::Vector3;

use crate::{
    channels::{self, Sub, Ch, Pub},
    functions::{self, map, variant_eq},
    sbus_cmd::{TriSwitch, SbusCmd},
    task_attitude_controller::StabilizationMode::*,
    task_blinker::BlinkerMode,
    task_flight_detector::FlightMode,
    task_motor_governor::{ArmedState, DisarmReason, MotorState}, config::definitions::Configuration,
};

use bitflags::bitflags;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct EnableIntFlag: u8 {
        const StabModeChange    = 1 << 0;
        const SbusFailSafe      = 1 << 1;
        const FlightModeChange  = 1 << 2;
        const MotorArmChange    = 1 << 3;
    }
}

static TASK_ID: &str = "[COMMANDER]";

#[embassy_executor::task]
pub async fn commander(
    spawner: Spawner,
    config: &'static Configuration,
    mut s_sbus_cmd: channels::SbusCmdSub,
    mut s_motor_state: channels::MotorStateSub,
    mut s_attitude_sense: channels::AttitudeSenseSub,
    mut s_flight_mode: channels::FlightModeSub,
    p_motor_arm: channels::MotorArmPub,
    p_motor_dir: channels::MotorDirPub,
    p_thrust_cmd: channels::ThrustActuatePub,
    p_blinker_mode: channels::BlinkerModePub,
    p_attitude_int_reset: channels::AttitudeIntEnablePub,
    p_attitude_stab_mode: channels::AttitudeStabModePub,
    p_motor_spin_check: channels::MotorSpinCheckPub,
    p_do_gyro_cal: channels::DoGyroCalPub,
    p_do_mag_cal: channels::DoMagCalPub,
) {

    p_motor_dir.publish_immediate(config.motor_dir);
    p_blinker_mode.publish_immediate(BlinkerMode::OnOffFast);

    let mut ref_yaw = 0.0;
    let mut current_stab_mode = Inactive;
    let mut current_motor_state = MotorState::Disarmed(DisarmReason::NotInitialized);
    let mut saved_flight_mode = FlightMode::Ground;
    
    let mut enable_con_int_flag = EnableIntFlag::empty();

    let p_gesture_detector = unwrap!(GD_SBUS.publisher());
    let mut s_gesture_detector = unwrap!(GESTURE_DETECTED.subscriber());

    spawner.must_spawn(stick_gestures(
        unwrap!(GD_SBUS.subscriber()),
        unwrap!(GESTURE_DETECTED.publisher())
    ));

    info!("{}: Entering main loop", TASK_ID);
    loop {

        // Handle RC inputs
        match s_sbus_cmd.next_message_pure().await {
            // Valid command received, send arming switch
            Ok(cmd) => {

                p_gesture_detector.publish_immediate(cmd);

                enable_con_int_flag.set(EnableIntFlag::SbusFailSafe, false);

                // Arm motors based on switch B
                p_motor_arm.publish_immediate(!cmd.sw_b.is_idle());

                // Engage motor spin test routine based on switch E
                p_motor_spin_check.publish_immediate(
                    cmd.sw_e.is_active() && saved_flight_mode == FlightMode::Ground,
                );

                // Modify thrust mode based on switch C
                let thrust = match cmd.sw_c {
                    TriSwitch::Idle => map(thrust_curve(cmd.thrust), 0., 1., 400., 1000.),
                    TriSwitch::Middle => map(thrust_curve(cmd.thrust), 0., 1., 250., 1500.),
                    TriSwitch::Active => map(thrust_curve(cmd.thrust), 0., 1., 250., 2047.),
                };

                p_thrust_cmd.publish_immediate(thrust);

                let stab_mode = match cmd.sw_f {
                    TriSwitch::Idle | TriSwitch::Middle => {
                        ref_yaw = functions::wrap(ref_yaw + cmd.yaw * 0.01, -PI, PI);
                        Horizon(Vector3::new(
                            roll_curve(cmd.roll),
                            pitch_curve(-cmd.pitch),
                            yaw_curve(ref_yaw),
                        ))
                    }
                    TriSwitch::Active => Acro(
                        Vector3::new(
                            roll_curve(cmd.roll * 10.),
                            pitch_curve(cmd.pitch * 10.),
                            yaw_curve(-cmd.yaw * 5.),
                        )
                    ),
                };

                p_attitude_stab_mode.publish_immediate(stab_mode);

                // Check if stabilization mode has changed or is uninitialized
                enable_con_int_flag.set(EnableIntFlag::StabModeChange, !variant_eq(&current_stab_mode, &stab_mode));
                current_stab_mode = stab_mode;
            }

            // SBUS failsafe, connection to remote control lost
            Err(error) => {
                p_motor_arm.publish_immediate(false);
                enable_con_int_flag.set(EnableIntFlag::SbusFailSafe, true);
                error!("{}: SBUS error -> {}", TASK_ID, error);
                Timer::after(Duration::from_hz(5)).await;
            }
        }

        // Disable and reset integral controller if craft is on the ground
        if let Some(flight_mode) = s_flight_mode.try_next_message_pure() {
            enable_con_int_flag.set(EnableIntFlag::FlightModeChange, flight_mode == FlightMode::Ground);
            saved_flight_mode = flight_mode;
        }

        // Disable and reset integrals controller if motors was just armed
        if let Some(motor_state) = s_motor_state.try_next_message_pure() {
            let motor_state_changed = !variant_eq(&current_motor_state, &motor_state);
            enable_con_int_flag.set(EnableIntFlag::MotorArmChange, motor_state_changed);
            current_motor_state = motor_state;
        }

        // Map motor states to LED blinker
        p_blinker_mode.publish_immediate(match current_motor_state {
            MotorState::Armed(ArmedState::Idle) => BlinkerMode::OnOffFast,
            MotorState::Armed(ArmedState::Running(_)) => BlinkerMode::TwoFast,
            MotorState::Disarmed(_) => BlinkerMode::OnOffSlow,
        });

        // Signal to enable or disable controller integrals based on error flag
        p_attitude_int_reset.publish_immediate( enable_con_int_flag.is_empty() );

        // Reset global references state value
        // NOTE - GPS position, altitude should go here as well
        if ! enable_con_int_flag.is_empty() {
            ref_yaw = s_attitude_sense.next_message_pure().await.0.z;
        }

        // Detect stick gestures and do proper callback
        if let Some(gesture) = s_gesture_detector.try_next_message_pure() {
            if saved_flight_mode == FlightMode::Ground {
                match gesture {
                    StickGesture::BottomInward  => p_do_gyro_cal.publish_immediate(true),
                    StickGesture::BottomRight   => p_do_mag_cal.publish_immediate(true),
                    _ => {},
                }
            }
        }
    }
}

fn pitch_curve(input: f32) -> f32 {
    input * 0.5
}

fn roll_curve(input: f32) -> f32 {
    input * 0.5
}

fn yaw_curve(input: f32) -> f32 {
    input
}

fn thrust_curve(input: f32) -> f32 {
    input
}

#[derive(Debug,PartialEq,Clone,Copy)]
enum StickGesture {
    BottomInward,
    BottomOutward,
    BottomRight,
    BottomLeft,
    None
}

///*
/// This file should be used as a template for new tasks 
///*/


static GD_SBUS: Ch<SbusCmd,1> = PubSubChannel::new();
static GESTURE_DETECTED: Ch<StickGesture,1> = PubSubChannel::new();

static TASK2_ID : &str = "[STICK_GESTURES]";

#[embassy_executor::task]
pub async fn stick_gestures(
    mut s_sbus_command: Sub<SbusCmd,1>,
    p_ground_gestures: Pub<StickGesture,1>,
) {

    let mut ticker = Ticker::every(Duration::from_hz(10));
    
    let mut last_region = StickGesture::None;
    let mut time_entered_region = Instant::now();
    
    info!("{}: Entering main loop",TASK2_ID);
    loop {

        let cmd = s_sbus_command.next_message_pure().await;

        // Match stick regions to specific gestures
        let region = match (cmd.thrust,cmd.yaw,cmd.pitch,cmd.roll) {
            (t,y,p,r) if t <  0.1 && y < -0.8 && p < -0.8 && r < -0.8 => StickGesture::BottomLeft,
            (t,y,p,r) if t <  0.1 && y >  0.8 && p < -0.8 && r < -0.8 => StickGesture::BottomInward,
            (t,y,p,r) if t <  0.1 && y >  0.8 && p < -0.8 && r >  0.8 => StickGesture::BottomRight,
            (t,y,p,r) if t <  0.1 && y < -0.8 && p < -0.8 && r >  0.8 => StickGesture::BottomOutward,
            (_,_,_,_) => StickGesture::None
        };

        // Check if sticks have been in same region for at least 1 second
        if region != StickGesture::None && region == last_region {
            if Instant::now().duration_since(time_entered_region) > Duration::from_secs(1) {

                info!("{}: Now doing gesture: {}",TASK2_ID,Debug2Format(&region));
                
                // Publish gesture
                p_ground_gestures.publish_immediate(region);

                // Wait for 2 seconds before recognizing new gestures
                Timer::after(Duration::from_secs(2)).await;
            }
        } else {
            time_entered_region = Instant::now()
        }

        last_region = region;
        ticker.next().await;
    }
}