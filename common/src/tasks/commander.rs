use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};

use crate::{calibration::{AccCalib, Calibrate, GyrCalib, MagCalib}, signals as s, types::control::ControlMode};

use super::motor_test::MotorTest;

pub enum CmdRequest {
    MotorTest(MotorTest),
    ArmMotors{arm: bool, force: bool},
    SetMotorSpeeds([f32; 4]),
    SetMode(ControlMode),
    CalibrateAcc((AccCalib, Option<u8>)),
    CalibrateGyr((GyrCalib, Option<u8>)),
    CalibrateMag((MagCalib, Option<u8>)),
}

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "commander";
    info!("{}: Task started", ID);

    // Task inputs
    let rcv_commander_request = s::COMMANDER_REQUEST.receiver();
    let mut rcv_arming_blocker = unwrap!(s::ARMING_BLOCKER.receiver());
    let mut rcv_motors_state = unwrap!(s::MOTORS_STATE.receiver());

    // Task outputs
    let snd_control_mode = s::CONTROL_MODE.sender();
    let snd_ctrl_motors = s::CTRL_MOTORS.sender();
    let snd_arm_motors = s::CMD_ARM_MOTORS.sender();
    let snd_calibrate = s::CMD_CALIBRATE.sender();

    // TODO Estimate if we are on the ground
    let on_ground = true;

    // DEBUG
    snd_control_mode.send(crate::types::control::ControlMode::Angle);

    let mut ticker = Ticker::every(Duration::from_hz(20));

    info!("{}: Entering main loop", ID);
    loop {
        match select(rcv_commander_request.receive(), ticker.next()).await {
            // This thing runs when a request is received
            Either::First(request) => match request {
                CmdRequest::MotorTest(test) => {
                    info!("{}: Received motor test request: {:?}", ID, test);
                    crate::tasks::motor_test::SIGNAL.signal(test);
                },
                CmdRequest::ArmMotors{arm, force} => {
                    debug!("{}: Received arm motors request: arm = {}, force = {}", ID, arm, force);
                    let blocker = rcv_arming_blocker.get().await;
                    if force || blocker.is_empty() {
                        snd_arm_motors.send((arm, force));
                    } else {
                        warn!("{}: Arming is blocked by {:?}", ID, blocker);
                    }
                },
                CmdRequest::SetMotorSpeeds(speeds) => {
                    if on_ground {
                        snd_ctrl_motors.send(speeds);
                    } else {
                        warn!("{}: Motor speeds can only be overwritten on the ground", ID);
                    }
                },
                CmdRequest::SetMode(mode) => snd_control_mode.send(mode),
                CmdRequest::CalibrateAcc(cal) => {
                    if !on_ground {
                        warn!("{}: Acc calibration is only possible on the ground", ID);
                        continue;
                    }

                    if !rcv_motors_state.get().await.is_disarmed() {
                        warn!("{}: Acc calibration is only possible when disarmed", ID);
                        continue;
                    }
                    snd_calibrate.send(Calibrate::Acc(cal))
                },
                CmdRequest::CalibrateGyr(cal) => {
                    if !on_ground {
                        warn!("{}: Gyro calibration is only possible on the ground", ID);
                        continue;
                    }

                    if !rcv_motors_state.get().await.is_disarmed() {
                        warn!("{}: Gyro calibration is only possible when disarmed", ID);
                        continue;
                    }

                    snd_calibrate.send(Calibrate::Gyr(cal))
                },
                CmdRequest::CalibrateMag(cal) => {
                    if !on_ground {
                        warn!("{}: Mag calibration is only possible on the ground", ID);
                        continue;
                    }

                    if !rcv_motors_state.get().await.is_disarmed() {
                        warn!("{}: Mag calibration is only possible when disarmed", ID);
                        continue;
                    }

                    snd_calibrate.send(Calibrate::Mag(cal))
                }
            },

            // These things run periodically
            Either::Second(_) => {

            }
        }
    }
}
