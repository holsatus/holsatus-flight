use defmt::*;
use embassy_time::{Ticker, Duration};

use crate::{common::types::{ArmedState, MotorState}, messaging as msg};

const TAKEOFF_ESTIMATOR_THRUST: u16 = 700;
static TASK_ID : &str = "[FLIGHT_DETECTOR]";

#[embassy_executor::task]
pub async fn flight_detector() {

    // Input messages
    let mut rcv_motor_state = msg::MOTOR_STATE.receiver().unwrap();
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();

    // Output messages
    let snd_landed_state = msg::LANDED_STATE.sender();

    // Send initial state
    snd_landed_state.send(LandedState::Undefined);
    
    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        
        // It is okay to unwrap here since we know the value is initialized above
        let mut landed_state = defmt::unwrap!(snd_landed_state.try_get());
        
        let motor_state = rcv_motor_state.changed().await;
        let imu_data = rcv_imu_data.changed().await;

        match landed_state {
            LandedState::Undefined => {
                if let MotorState::Disarmed(_) = motor_state {
                    landed_state = LandedState::OnGround;
                    info!("{}: Changing flight mode to: {}",TASK_ID,landed_state);
                }
            },
            LandedState::OnGround => {
                if let MotorState::Armed(ArmedState::Running(m)) = motor_state {
                    let [a,b,c,d] = m;
                    let z_thrust = (a+b+c+d)/4;
                    if z_thrust > TAKEOFF_ESTIMATOR_THRUST && imu_data.acc.z < -12.0 {
                        landed_state = LandedState::InAir;
                        info!("{}: Changing flight mode to: {}",TASK_ID,landed_state);
                    }
                }
            },
            LandedState::InAir => {
                if let MotorState::Disarmed(_) = motor_state {
                    landed_state = LandedState::OnGround;
                    info!("{}: Changing flight mode to: {}",TASK_ID,landed_state);
                }
            },
            mode => {
                defmt::error!("{}: The flight mode '{}' should not be reachable!", mode,TASK_ID);
            }
        }

        // Only transmit if the flight mode has changed
        if snd_landed_state.try_get_and(|f| f != &landed_state).is_some() {
            snd_landed_state.send(landed_state);
        }
        
        ticker.next().await;

    }
}


/// Based off the MavlLink specification
/// https://mavlink.io/en/messages/common.html#MAV_LANDED_STATE
#[derive(Clone, Copy, PartialEq, Format)]
pub enum LandedState {
    Undefined,
    OnGround,
    InAir,
    Takeoff,
    Landing,
}
