use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Ticker};

use crate::{calibration::{AccCalib, Calibrate, GyrCalib, MagCalib}, signals as s, sync::rpc::Procedure, types::control::ControlMode};

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

#[derive(Debug)]
pub enum Origin {
    RemoteCtl,
    GroundCtl,
    Offboard,
}

crate::rpc_message!{
    CommanderRpc: Request -> Response,

    /// Request to arm or disarm the motors
    #[derive(Debug)]
    async ArmMotors {
        arm: bool,
        force: bool
    } -> bool,

    async SetActuators {
        speeds: [f32; 4]
    } -> bool,
}

pub static COMMANDER_PROC: Procedure<CommanderRpc> = Procedure::new();

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "commander";
    info!("{}: Task started", ID);

    // Task inputs
    let rcv_commander_request = s::COMMANDER_REQUEST.receiver();
    let mut rcv_arming_blocker = s::ARMING_BLOCKER.receiver();
    let mut rcv_motors_state = s::MOTORS_STATE.receiver();

    // Task outputs
    let mut snd_control_mode = s::CONTROL_MODE.sender();
    let mut snd_ctrl_motors = s::CTRL_MOTORS.sender();
    let mut snd_arm_motors = s::CMD_ARM_MOTORS.sender();
    let mut snd_calibrate = s::CMD_CALIBRATE.sender();

    // TODO Estimate if we are on the ground
    let on_ground = true;

    // DEBUG
    snd_control_mode.send(crate::types::control::ControlMode::Angle);

    let mut ticker = Ticker::every(Duration::from_hz(20));

    info!("{}: Entering main loop", ID);
    loop {
        match select3(rcv_commander_request.receive(), COMMANDER_PROC.get_request(), ticker.next()).await {
            // This thing runs when a request is received
            Either3::First(request) => match request {
                CmdRequest::MotorTest(test) => {
                    info!("{}: Received motor test request: {:?}", ID, test);
                    crate::tasks::motor_test::SIGNAL.signal(test);
                },
                CmdRequest::ArmMotors{arm, force} => {
                    debug!("{}: Received arm motors request: arm = {}, force = {}", ID, arm, force);
                    let blocker = rcv_arming_blocker.get().await;
                    if force || !arm || blocker.is_empty() {
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

            // These handle procedure requests
            Either3::Second(procedure) => match procedure {
                Request::ArmMotors(handle, req) => {
                    let (arm, force) = (req.arm, req.force);
                    debug!("{}: Received arm motors request: arm = {}, force = {}", ID, arm, force);

                    // Retrieve the arming blocker flag
                    use crate::tasks::arm_blocker::get_arming_blocker;
                    let blocker = get_arming_blocker().await;

                    if force || !arm || blocker.is_empty() {
                        snd_arm_motors.send((arm, force));
                        handle.respond(true);
                    } else {
                        warn!("{}: Arming is blocked by {:?}", ID, blocker);
                        handle.respond(false);
                    }
                }
                Request::SetActuators(handle, req) => {
                    if on_ground && req.speeds.iter().all(|&s| s >= 0.0 && s <= 1.0) {
                        snd_ctrl_motors.send(req.speeds);
                        handle.respond(true);
                    } else {
                        handle.respond(false);
                    }
                }
            },

            // These things run periodically
            Either3::Third(_) => {

            }
}
    }
}
