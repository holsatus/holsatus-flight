use defmt::*;
use embassy_futures::select::{select, Either};
use embassy_time::{Timer, Duration};
use nalgebra::Vector3;

use crate::channels;

static TASK_ID : &str = "MOTOR_MIXING";


#[embassy_executor::task]
pub async fn motor_mixing(
    mut s_thrust_actuate: channels::ThrustActuateSub,
    mut s_attitude_actuate: channels::AttitudeActuateSub,
    mut s_motor_arm: channels::MotorArmSub,
    mut s_motor_spin_check: channels::MotorSpinCheckSub,
    p_motor_speed: channels::MotorSpeedPub,
) {

    let mut arm_motors = false;

    let mut attitude = Vector3::new(0., 0., 0.);
    let mut thrust = 0.0;

    info!("{} : Entering main loop",TASK_ID);
    loop {
        
        match select(s_attitude_actuate.next_message_pure(), s_thrust_actuate.next_message_pure()).await {
            Either::First(a) => attitude = a,
            Either::Second(t) => thrust = t,
        }

        let command = motor_mixing_matrix_inverted(thrust, attitude, 70., 2047.);

        if let Some(arm) = s_motor_arm.try_next_message_pure() {
            arm_motors = arm
        }

        match arm_motors {
            true => {
                if let Some(true) = s_motor_spin_check.try_next_message_pure() {
                    motor_spin_check_routine(&p_motor_speed).await;
                } else {
                    p_motor_speed.publish_immediate(Some(command))
                }
            },
            false => p_motor_speed.publish_immediate(None)
        }
    }
}

/// Basic quad-copter motor mixing function, for quads using the standard X configuration, shown below
///
/// `M3( CW) ^ M1(CCW)`
/// 
/// `Left ------ Right`
/// 
/// `M2(CCW) | M4( CW)`
#[allow(unused)]
fn motor_mixing_matrix(thrust: f32, attitude: Vector3<f32>, min: f32, max: f32) -> [u16;4] {
    [
        (thrust - attitude.x + attitude.y + attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x - attitude.y + attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x + attitude.y - attitude.z).clamp(min, max) as u16,
        (thrust - attitude.x - attitude.y - attitude.z).clamp(min, max) as u16,
    ]
}

/// Basic quad-copter motor mixing function, for quads using the inverted X configuration, shown below
///
/// `M3(CCW) ^ M1( CW)`
/// 
/// `Left ------ Right`
/// 
/// `M2( CW) | M4(CCW)`
#[allow(unused)]
fn motor_mixing_matrix_inverted(thrust: f32, attitude: Vector3<f32>, min: f32, max: f32) -> [u16;4] {
    [
        (thrust - attitude.x + attitude.y - attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x - attitude.y - attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x + attitude.y + attitude.z).clamp(min, max) as u16,
        (thrust - attitude.x - attitude.y + attitude.z).clamp(min, max) as u16,
    ]
}

async fn motor_spin_check_routine(p_motor_speed: &channels::MotorSpeedPub) {

    // Spin motor 1
    for _ in 0..100 {
        p_motor_speed.publish_immediate(Some([70,0,0,0]));
        Timer::after(Duration::from_hz(100)).await;
    }

    // Spin motor 3
    for _ in 0..100 {
        p_motor_speed.publish_immediate(Some([0,70,0,0]));
        Timer::after(Duration::from_hz(100)).await;
    }

    // Spin motor 3
    for _ in 0..100 {
        p_motor_speed.publish_immediate(Some([0,0,70,0]));
        Timer::after(Duration::from_hz(100)).await;
    }

    // Spin motor 4
    for _ in 0..100 {
        p_motor_speed.publish_immediate(Some([0,0,0,70]));
        Timer::after(Duration::from_hz(100)).await;
    }

    // Wait 1 second
    for _ in 0..100 {
        p_motor_speed.publish_immediate(Some([0,0,0,0]));
        Timer::after(Duration::from_hz(100)).await;
    }
}