use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant};
use nalgebra::{Quaternion, Unit, Vector3, Vector4};
use sbus::SBusPacket;

use crate::{
    airframe::MotorMixing, common::types::{AttitudeReference, MotorState, VehicleState}, config::{AttitudePids, DshotSpeed, ImuConfig, MagConfig}, sensors::imu::types::ImuData6Dof, t_flight_detector::LandedState, t_motor_governor::ArmBlocker, t_sbus_reader::RxError, transmitter::{ControlRequest, EventRequest, TransmitterMap}
};

macro_rules! watch {
    ($name:ident, $datatype:ty, $num:literal) => {
        watch!($name, $datatype, $num, ThreadModeRawMutex, "Watch channel");
    };
    ($name:ident, $datatype:ty, $num:literal, $doc:expr) => {
        watch!($name, $datatype, $num, ThreadModeRawMutex, $doc);
    };
    ($name:ident, $datatype:ty, $num:literal, $mutex:ident) => {
        watch!($name, $datatype, $num, $mutex, "Watch channel");
    };
    ($name:ident, $datatype:ty, $num:literal, $mutex:ident, $doc:expr) => {
        #[doc = $doc]
        pub static $name: embassy_sync::watch::Watch<embassy_sync::blocking_mutex::raw::$mutex, $datatype, $num> = embassy_sync::watch::Watch::new();
    };
}

// Shorthand for the watch and channel types
type SWatch<T, const N: usize> = embassy_sync::watch::Watch<ThreadModeRawMutex, T, N>;


// Best effort IMU and MAG data, selected by the respective governors
watch!(IMU_DATA, ImuData6Dof, 3, "Best effort IMU data, selected by the IMU governor task.");
watch!(MAG_DATA, Vector3<f32>, 3, "Best effort MAG data, selected by the MAG governor task.");

/// Array of watch channels for the `N_IMU` individual IMU sensors. These should generally not be used, as the
/// IMU governor task is responsible for selecting the active sensor, transmitted through the `IMU_DATA` channel.
pub static IMU_SENSOR: [SWatch<(ImuData6Dof, Instant), 3>; crate::N_IMU] = [IMU_SENSOR_REPEAT; crate::N_IMU];
const IMU_SENSOR_REPEAT: SWatch<(ImuData6Dof, Instant), 3> = SWatch::new();

/// Array of watch channels for the `N_MAG` individual MAG sensors. These should generally not be used, as the
/// MAG governor task is responsible for selecting the active sensor, transmitted through the `MAG_DATA` channel.
pub static MAG_SENSOR: [SWatch<(Vector3<f32>, Instant), 3>; crate::N_MAG] = [MAG_SENSOR_REPEAT; crate::N_MAG];
const MAG_SENSOR_REPEAT: SWatch<(Vector3<f32>, Instant), 3> = SWatch::new();

// Indicates the ID of the active IMU and MAG sensor
watch!(IMU_ACTIVE_ID, Option<u8>, 2, "The ID of the active IMU sensor, selected by the IMU governor task.");
watch!(MAG_ACTIVE_ID, Option<u8>, 2, "The ID of the active MAG sensor, selected by the MAG governor task.");

// Flag indicating whether the individual sensors are currently calibrating
watch!(GYR_CALIBRATING, bool, 3, "This channel is set by the gyroscope calibration task when the calibration process is active.");
watch!(ACC_CALIBRATING, bool, 3, "This channel is set by the accelerometer calibration task when the calibration process is active.");
watch!(MAG_CALIBRATING, bool, 3, "This channel is set by the magnetometer calibration task when the calibration process is active.");

// Flag indicating whether the individual sensors are calibrated
watch!(GYR_CALIBRATED, bool, 2, "Signals whether the gyroscopes are calibrated");
watch!(ACC_CALIBRATED, bool, 2, "Signals whether the accelerometers are calibrated");
watch!(MAG_CALIBRATED, bool, 2, "Signals whether the magnetometers are calibrated");

// Command to start the calibration process for the individual sensors
watch!(CMD_START_GYR_CALIB, bool, 2, "Command to start calibrating the gyroscopes");
watch!(CMD_START_ACC_CALIB, bool, 2, "Command to start calibrating the accelerometers");
watch!(CMD_START_MAG_CALIB, bool, 2, "Command to start calibrating the magnetometers");

// Various commands to the system, set by the commander task
watch!(CMD_SAVE_CONFIG, bool, 2, "Command to commit any changes to the configuration to flash");
watch!(CMD_ARM_VEHICLE, bool, 2, "Command to arm or disarm the vehicle, sent by rc_mapper");
watch!(CMD_EN_INTEGRAL, bool, 2, "Channel which can enable or disable integral terms of the attitude controller");
watch!(CMD_ATTITUDE_SETPOINT, AttitudeReference, 2, "The target throttle for the motor mixing");
watch!(CMD_THROTTLE_SETPOINT, f32, 2, "The target throttle for the motor mixing");

// Channels used for remote control
watch!(SBUS_DATA, Result<SBusPacket, RxError>, 3, "Data from the SBUS receiver task");
watch!(REQUEST_CONTROLS, ControlRequest, 2, "The target attitude (angle or rate) for the attitude controller");
/// Queue of discrete system requests, such as sensor calibration, configuration changes, etc.
pub static REQUEST_QUEUE: Channel<ThreadModeRawMutex, EventRequest, 20> = Channel::new();

// Channel for various vehicle states
watch!(VEHICLE_STATE, VehicleState, 3, "Channel containing the vehicle state, selected by the vehicle state governor task.");
watch!(ARM_BLOCKER, ArmBlocker, 2, "Arming blocker flag, set by the arming checker task, describes whether it is safe to arm the vehicle");
watch!(LANDED_STATE, LandedState, 2, "Landed state, set by the flight detector, describes whether the vehicle is landed, airborne or in transition");

watch!(ATTITUDE_QUAT, Unit<Quaternion<f32>>, 2, "The current vehicle attitude represented as a quaternion");
watch!(ATTITUDE_EULER, Vector3<f32>, 4, "The current vehicle attitude represented as euler angles");
watch!(CONTROLLER_OUTPUT, Vector3<f32>, 2, "The output of the attitude controllers for each axis");
watch!(MOTORS_MIXED, Vector4<u16>, 2, "The mixed controller output and thrust target, using a mixing matrix");
watch!(LOOP_HEALTH, f32, 2, "Loop health transmitted by the attitude controller, is 1.0 when the loop is healthy");
watch!(MOTOR_SPEEDS, [u16; 4], 2, "Motor speeds, as decided by the attitude controller, transmitted to the motor governor");
watch!(MOTOR_STATE, MotorState, 2, "Motor state, set by the motor governor, describes the arming state and current speed of the motors");

// Signals which represent configurations within the system. These are initialized by the configuration governor task,
// but are subsequently supposed to only by other tasks, such as calibration tasks.
watch!(CFG_IMU_CONFIG, [Option<ImuConfig>; crate::N_IMU], 3, "Transmits the configurations for the individual IMU sensors");
watch!(CFG_MAG_CONFIG, [Option<MagConfig>; crate::N_IMU], 3, "Transmits the configurations for the individual MAG sensors");
watch!(CFG_MIXING_MATRIX, MotorMixing, 2, "The motor mixing matrix, used to convert controller outputs to motor speeds");
watch!(CFG_LOOP_FREQUENCY, u16, 2, "The desired frequency of the attitude control loop in Hz.");
watch!(CFG_MAG_FREQUENCY, u16, 2, "The desired sampling rate of the magnetometer in Hz.");
watch!(CFG_ATTITUDE_PIDS, AttitudePids, 2, "Configurations for the attitude controllers");
watch!(CFG_REVERSE_MOTOR, [bool;4], 2, "Sets the spin direction of each motor");
watch!(CFG_DSHOT_SPEED, DshotSpeed, 2, "Speed setting for the DSHOT protocol");
watch!(CFG_DSHOT_TIMEOUT, Duration, 2, "Sets the timeout of the motor governor");
watch!(CFG_RADIO_TIMEOUT, Duration, 2, "Sets the timeout of the RC radio receiver");
watch!(CFG_TRANSMITTER_MAP, TransmitterMap, 2, "The transmitter map, containing the mapping of the transmitter channels to vehicle commands");
