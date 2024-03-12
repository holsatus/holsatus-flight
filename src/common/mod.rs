use bitflags::bitflags;

use self::types::MotorState;

mod mav_bitflag;
pub mod rotation_matrices;
pub mod types;

bitflags! {
    /// High latency telemetry failure flags
    ///
    /// https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
    pub struct HlFailureFlag: u16 {
        const GPS               = 0b_00000000000001;
        const DIF_PRESSURE      = 0b_00000000000010;
        const ABS_PRESSURE      = 0b_00000000000100;
        const ACCELEROMETER_3D  = 0b_00000000001000;
        const GYROSCOPE_3D      = 0b_00000000010000;
        const MAGNETOMETER_3D   = 0b_00000000100000;
        const TERRAIN           = 0b_00000001000000;
        const BATTERY           = 0b_00000010000000;
        const RC_RECEIVER       = 0b_00000100000000;
        const OFFBOARD_LINK     = 0b_00001000000000;
        const ENGINE            = 0b_00010000000000;
        const GEOFENCE          = 0b_00100000000000;
        const ESTIMATOR         = 0b_01000000000000;
        const MISSION           = 0b_10000000000000;
    }
}


// TODO Move to a more appropriate location
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SimpleRequest {

    /// Unbound request // TODO remove?
    Unbound,

    /// Arm all motors
    ArmMotors,

    /// Disarm all motors. Acts as a kill-switch
    DisarmMotors,

    /// Change stabilization mode to angle (or horizon) mode
    AngleMode,

    /// Change stabilization mode to rate (or acro / 3D) mode
    RateMode,

    /// Start gyroscope calibration
    StartGyrCalib,

    /// Abort any currently active gyroscope calibration
    AbortGyrCalib,

    /// Start accelerometer calibration
    StartAccCalib,

    /// Abort any currently active accelerometer calibration
    AbortAccCalib,

    /// Start magnetometer calibration
    StartMagCalib,

    /// Abort any currently active magnetometer calibration
    AbortMagCalib,

    /// Save any currently modified parameters to flash
    SaveConfig,

    /// RC Failsafe
    RcFailsafe,
}

pub enum EventRequest {
    ArmMotors(bool),
    SetAngleMode(bool),
    StartGyrCalib{seconds: u8, timeout: bool, variance: f32},
    AbortGyrCalib,
    StartAccCalib{seconds: u8, timeout: bool, variance: f32},
    AbortAccCalib,
    SaveConfig,
}

pub enum EventResponse {
    ArmMotors(Result<MotorState, ArmMotorsError>),
    SetAngleMode(Result<(),SetAngleModeError>),
    StartGyrCalib(Result<[Option<Result<f32, GyrCalibError>>; crate::N_IMU], StartGyrCalibError>),
    AbortGyrCalib(Result<[Option<Result<f32, GyrCalibError>>; crate::N_IMU], StartAccCalibError>),
    StartAccCalib(StartAccCalibError),
    AbortAccCalib(AbortAccCalibError),
    SaveConfig,
}

pub struct GyrCalibError {}
pub struct AccCalibError {}

pub enum ArmMotorsError {
    Ok(MotorState),
    NotSafeToArm,
}

pub enum SetAngleModeError {
    NotInManualMode,
}

pub enum StartGyrCalibError {
    NotSafeToCalibrate,
    NotCalibrating,
}

pub enum StartAccCalibError {
    NotSafeToCalibrate,
    NotCalibrating,
}

pub enum AbortAccCalibError {
    NotCalibrating,
}

pub enum SaveConfigError {
    NotSafeToSave,
}