use embassy_time::{Duration, Ticker};
use crate::{common::types::VehicleState, messaging as msg};

#[embassy_executor::task]
pub async fn vehicle_state() -> ! {
    // Input channels
    let mut rcv_gyr_calibrating = msg::GYR_CALIBRATING.receiver().unwrap();
    let mut rcv_acc_calibrating = msg::ACC_CALIBRATING.receiver().unwrap();
    let mut rcv_landed_state = msg::LANDED_STATE.receiver().unwrap();

    // Output channels
    let snd_vehicle_state = msg::VEHICLE_STATE.sender();

    // Initialize the vehicle state as being uninitialized
    snd_vehicle_state.send(VehicleState::Uninit);

    // REVIEW - This entire task could be housed in the commander instead

    let mut ticker = Ticker::every(Duration::from_hz(10));

    '_infinite: loop {

        let mut calibrating = false;
        let mut airborne = false;

        if let Some(gyr_calibrating) = rcv_gyr_calibrating.try_changed() {
            calibrating |= gyr_calibrating;
        }

        if let Some(acc_calibrating) = rcv_acc_calibrating.try_changed() {
            calibrating |= acc_calibrating;
        }

        if let Some(landed_state) = rcv_landed_state.try_changed() {
            use crate::t_flight_detector::LandedState as L;
            match landed_state {
                L::Undefined | L::OnGround => airborne = false,
                L::Takeoff | L::Landing | L::InAir => airborne = true,
            }
        }

        if calibrating {
            snd_vehicle_state.send(VehicleState::Calibrating);
        } else if airborne {
            snd_vehicle_state.send(VehicleState::Active);
        } else {
            snd_vehicle_state.send(VehicleState::Standby);
        }

        ticker.next().await;

    }
}