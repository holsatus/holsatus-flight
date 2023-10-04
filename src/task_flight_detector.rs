use defmt::*;
use embassy_time::{Ticker, Duration};
use crate::{channels, config};

use crate::task_motor_governor::{MotorState, ArmedState, DisarmReason};

static TASK_ID : &str = "[FLIGHT_DETECTOR]";

#[embassy_executor::task]
pub async fn flight_detector(
    mut s_motor_state: channels::MotorStateSub,
    p_flight_mode: channels::FlightModePub,
) {

    let mut flight_mode = FlightMode::Ground;
    let mut motor_state = MotorState::Disarmed(DisarmReason::NotInitialized);
    
    let mut ticker = Ticker::every(Duration::from_hz(10));
    info!("{}: Entering main loop",TASK_ID);
    loop {

        if let Some(s) = s_motor_state.try_next_message_pure() { motor_state = s }

        match flight_mode {
            FlightMode::Ground => {
                if let MotorState::Armed(ArmedState::Running(m)) = motor_state {
                    let [m1,m2,m3,m4] = m;
                    let z_thrust = (m1 + m2 + m3 + m4)/4; 
                    if z_thrust > config::definitions::TAKEOFF_ESTIMATOR_THRUST {
                        flight_mode = FlightMode::Flying;
                        info!("{}: Changing flight mode to: {}",TASK_ID,flight_mode);
                    }
                }
            },
            FlightMode::Flying => {
                if let MotorState::Disarmed(_) = motor_state {
                    flight_mode = FlightMode::Ground;
                    info!("{}: Changing flight mode to: {}",TASK_ID,flight_mode);
                }
            }
        }

        p_flight_mode.publish_immediate(flight_mode.clone());
        ticker.next().await;
        
    }
}

#[derive(Clone,Copy,PartialEq,Format)]
pub enum FlightMode {
    Ground,
    Flying
}