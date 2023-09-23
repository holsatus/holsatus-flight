use defmt::*;
use embassy_futures::select::{select, Either};
use nalgebra::Vector3;

use crate::channels;

static TASK_ID : &str = "MOTOR_MIXING";

#[embassy_executor::task]
pub async fn motor_mixing(
    mut s_sg_thrust_cmd: channels::ThrustActuateSub,
    mut s_sg_attitude_cmd: channels::AttitudeActuateSub,
    mut s_sg_motor_arm: channels::MotorArmSub,
    p_sg_motor_speed: channels::MotorSpeedPub,
) {

    let mut arm_motors = false;

    let mut attitude = Vector3::new(0., 0., 0.);
    let mut thrust = 0.0;

    info!("{} : Entering main loop",TASK_ID);
    loop {

        match select(s_sg_attitude_cmd.next_message_pure(), s_sg_thrust_cmd.next_message_pure()).await {
            Either::First(a) => attitude = a,
            Either::Second(t) => thrust = t,
        }

        let command = motor_mixing_matrix(thrust, attitude, 70., 2047.);

        if let Some(arm) = s_sg_motor_arm.try_next_message_pure() {
            arm_motors = arm
        }

        match arm_motors {
            true => p_sg_motor_speed.publish_immediate(Some(command)),
            false => p_sg_motor_speed.publish_immediate(None)
        }

    }
}

/// Basic quad-copter motor mixing function
fn motor_mixing_matrix(thrust: f32, attitude: Vector3<f32>, min: f32, max: f32) -> [u16;4] {
    [
        (thrust - attitude.x + attitude.y - attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x - attitude.y - attitude.z).clamp(min, max) as u16,
        (thrust + attitude.x + attitude.y + attitude.z).clamp(min, max) as u16,
        (thrust - attitude.x - attitude.y + attitude.z).clamp(min, max) as u16,
    ]
}