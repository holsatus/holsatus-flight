use defmt::{info, warn, Format};
use embassy_rp::peripherals::PIO0;
use embassy_time::{Duration, Timer, with_timeout};
use dshot_pio::{dshot_embassy_rp::DshotPio, DshotPioTrait};
use crate::channels;
use crate::cfg;

static TASK_ID : &str = "MOTOR_GOVERNOR";

/// Task to govern the arming, disarming and speed settings of the motors.
/// Arming takes 3.5 seconds: 3.0 s to arm, 0.5 s to set direction
#[embassy_executor::task]
pub async fn motor_governor(
    mut s_motor_speed: channels::MotorSpeedSub,
    mut s_motor_dir: channels::MotorDirSub,
    p_motor_state: channels::MotorStatePub,
    mut out_dshot_pio: DshotPio<'static,4,PIO0>,
) {

    let mut cfg_reverse_motor = s_motor_dir.next_message_pure().await;

    p_motor_state.publish_immediate(MotorState::Disarmed(DisarmReason::NotInitialized));

    loop {
        // Wait for signal to arm motors
        while s_motor_speed.next_message_pure().await.is_none() {}

        // Check if motor direction has been configured
        if let Some(r) = s_motor_dir.try_next_message_pure() {cfg_reverse_motor = r}

        // Send minimum throttle for a few seconds to arm the ESCs
        info!("{} : Initializing motors",TASK_ID);
        Timer::after(Duration::from_millis(500)).await;
        for _i in 0..50 {
            out_dshot_pio.throttle_minimum();
            Timer::after(Duration::from_millis(50)).await;
        }

        // Set motor directions for the four motors
        info!("{} : Setting motor directions",TASK_ID);
        for _i in 0..10 {
            out_dshot_pio.reverse(cfg_reverse_motor.into());
            Timer::after(Duration::from_millis(50)).await;
        }

        // Set motors armed signal
        p_motor_state.publish_immediate(MotorState::Armed(ArmedState::Idle));

        info!("{} : Entering main loop",TASK_ID);
        loop {

            match with_timeout(cfg::MOTOR_GOV_TIMEOUT, s_motor_speed.next_message_pure()).await {

                // Motor speed message received correctly
                Ok(Some(speeds)) => {
                    out_dshot_pio.throttle_clamp(speeds);
                    p_motor_state.publish_immediate(MotorState::Armed(ArmedState::Running(speeds)));
                },

                // Channel commanded motors to disarm
                Ok(None) =>  {
                    warn!("{} : Disarming motors -> commanded",TASK_ID);
                    out_dshot_pio.throttle_minimum();
                    p_motor_state.publish_immediate(MotorState::Disarmed(DisarmReason::Commanded));
                    break
                },

                // Automatic disarm due to message timeout
                Err(_) =>  {
                    warn!("{} : Disarming motors -> timeout",TASK_ID);
                    out_dshot_pio.throttle_minimum();
                    p_motor_state.publish_immediate(MotorState::Disarmed(DisarmReason::Timeout));
                    break
                }
            }
        }
    }
}

#[derive(Clone,Copy,Format,PartialEq)]
pub enum MotorState {
    Disarmed(DisarmReason),
    Armed(ArmedState)
}

#[derive(Clone,Copy,Format,PartialEq)]
pub enum ArmedState {
    Running([u16;4]),
    Idle
}

#[derive(Clone,Copy,Format,PartialEq)]
pub enum DisarmReason{
    NotInitialized,
    Commanded,
    Timeout
}