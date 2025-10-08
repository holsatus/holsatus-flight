use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Ticker};
use mutex::raw_impls::cs::CriticalSectionRawMutex;

use crate::{
    calibration::{AccCalib, Calibrate, GyrCalib, MagCalib},
    signals as s,
    sync::procedure::Procedure,
    types::control::ControlMode,
};

use super::motor_test::MotorTest;

pub enum CmdRequest {
    MotorTest(MotorTest),
    ArmMotors { arm: bool, force: bool },
    SetMotorSpeeds([f32; 4]),
    SetMode(ControlMode),
    CalibrateAcc((AccCalib, Option<u8>)),
    CalibrateGyr((GyrCalib, Option<u8>)),
    CalibrateMag((MagCalib, Option<u8>)),
}

/// The command to be sent to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    ArmDisarm {
        arm: bool,
        force: bool,
    },

    /// Start the calibration of the accelerometer(s)
    /// If `sensor_id` is `None`, all accelerometers will be calibrated
    /// otherwise only the accelerometer with the specified ID is calibrated
    ///
    /// Since this is a long-running operating handled by a separate task, the
    /// response will be `Response::Accepted` if the calibration was started
    /// successfully. The completion of the calibration will be indicated by
    /// a value produced by the calibrator.
    DoCalibration {
        sensor_id: Option<u8>,
        sensor_type: Sensor,
    },
    SetActuators {
        group: u8,
        values: [f32; 4],
    },
    SetActuatorOverride {
        active: bool,
    },
    ArmChecks,
}

/// A request to the [`Commander`](crate::commander::Commander)
pub struct Request {
    pub command: Command,
    pub origin: Origin,
}

// TODO Move to somewhere else
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sensor {
    Accelerometer,
    Gyroscope,
    Magnetometer,
    Barometer,
}

/// The response to a command to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    /// The command requested an unsupported operation
    Unsupported,

    /// Some necessary resources are not available
    Unavailble,

    /// The command would have no effect on the system
    Unchanged,

    /// The command was accepted and processed appropriately
    Accepted,

    /// The comnmand was rejected due to the current state of the system
    Rejected,

    /// The some error occured while processing the command
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Origin {
    /// The command was sent by the user via the RC
    RemoteControl,

    /// The command was sent via a telemetry link
    GroundControl,

    /// The command is of unspecified origin
    Unspecified,

    /// The command was the result of a failsafe event
    Failsafe,

    /// The command was the result of a kill switch event
    KillSwitch,

    /// The command was issued by an automated event
    Automatic,
}

pub static COMMANDER_PROC: Procedure<CriticalSectionRawMutex, Request, Response, 2> = Procedure::new();

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
        match select3(
            rcv_commander_request.receive(),
            COMMANDER_PROC.receive_request(),
            ticker.next(),
        )
        .await
        {
            // This thing runs when a request is received
            Either3::First(request) => match request {
                CmdRequest::MotorTest(test) => {
                    info!("{}: Received motor test request: {:?}", ID, test);
                    crate::tasks::motor_test::SIGNAL.signal(test);
                }
                CmdRequest::ArmMotors { arm, force } => {
                    debug!(
                        "{}: Received arm motors request: arm = {}, force = {}",
                        ID, arm, force
                    );
                    let blocker = rcv_arming_blocker.get().await;
                    if force || !arm || blocker.is_empty() {
                        snd_arm_motors.send((arm, force));
                    } else {
                        warn!("{}: Arming is blocked by {:?}", ID, blocker);
                    }
                }
                CmdRequest::SetMotorSpeeds(speeds) => {
                    if on_ground {
                        snd_ctrl_motors.send(speeds);
                    } else {
                        warn!("{}: Motor speeds can only be overwritten on the ground", ID);
                    }
                }
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
                }
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
                }
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
            Either3::Second((request, handle)) => match request.command {
                Command::ArmDisarm{ arm, force } => {
                    debug!(
                        "{}: Received arm motors request: arm = {}, force = {}",
                        ID, arm, force
                    );

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
            Either3::Third(_) => {}
        }
    }
}
