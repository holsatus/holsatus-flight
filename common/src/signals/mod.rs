use core::sync::atomic::{AtomicBool, AtomicU16, AtomicUsize};

use embassy_sync::{channel::Channel, once_lock::OnceLock, pubsub::{PubSubBehavior, PubSubChannel}};
use embassy_time::Duration;
use nalgebra::UnitQuaternion;

use crate::{
    calibration::{sens3d::Calib3DType, Calibrate}, errors::HolsatusError, filters::rate_pid::RatePidCfg3D, health::redundancy::Mode, rc_mapping::RcBindings, tasks::{calibrator::CalibratorState, commander::CmdRequest}, types::{
        actuators::MotorsState,
        blackbox::{LogPreset, LoggableType},
        config::{BootConfig, Vehicle},
        control::{ControlMode, RcAnalog},
        measurements::{GnssData, Imu6DofData, Imu9DofData},
        status::{ArmingBlocker, PidTerms, RcStatus},
    }, utils::rot_matrix::Rotation, NUM_IMU, NUM_MAG
};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as M;
use embassy_sync::watch::Watch as SyncWatch;

/// Helper macro to create multiple independent instances
/// of a watch e.g. for implementing redundant sensors.
macro_rules! multi_watch {
    ($name:ident, $type:ty, $count:ident, $subs:literal) => {
        pub static $name: [Watch<$type>; $count] = {
            const REPEAT: Watch<$type> = Watch::new();
            [REPEAT; $count]
        };
    };
}

#[macro_export]
macro_rules! multi_receiver {
    ($name:path, $count:ident) => {{
        let arr: [_; $count] = core::array::from_fn(|idx| $name[idx].receiver());
        arr
    }};
}

#[macro_export]
macro_rules! multi_sender {
    ($name:path, $count:ident) => {{
        let arr: [_; $count] = core::array::from_fn(|idx| $name[idx].sender());
        arr
    }};
}

// type Watch<T, const N: usize> = SyncWatch<M, T, N>;
use crate::sync::watch::Watch;

/// Channel for all system errors to be published to.
pub static ERR_CHANNEL: PubSubChannel<M, HolsatusError, 16, 2,0> = PubSubChannel::new();

/// Immediately publish an error to the global error channel.
pub fn register_error(error: impl Into<HolsatusError>) {
    ERR_CHANNEL.publish_immediate(error.into());
}

// Raw sensor data. These are not calibrated in any way!
// NOTE: For redundant sensors we need an array of these!
pub static RAW_IMU_DATA: Watch<Imu6DofData<f32>> = Watch::new();
pub static RAW_MAG_DATA: Watch<Imu9DofData<f32>> = Watch::new();

// Calibrated sensor data.
// NOTE: For redundant sensors we need an array of these!
pub static CAL_IMU_DATA: Watch<Imu6DofData<f32>> = Watch::new();
pub static CAL_MAG_DATA: Watch<Imu9DofData<f32>> = Watch::new();

/// Estimated vehicle attitude
pub static AHRS_ATTITUDE_Q: Watch<UnitQuaternion<f32>> = Watch::new();
pub static AHRS_ATTITUDE: Watch<[f32; 3]> = Watch::new();

/// Raw data packet from GNSS module
pub static RAW_GNSS_DATA: Watch<GnssData> = Watch::new();
pub static ESTIMATED_POS: Watch<[f32; 3]> = Watch::new();
pub static ESTIMATED_VEL: Watch<[f32; 3]> = Watch::new();

// Setpoint signals for the controller tasks
pub static TRUE_THROTTLE_SP: Watch<f32> = Watch::new();
pub static TRUE_VELOCITY_SP: Watch<[f32; 3]> = Watch::new();
pub static TRUE_ANGLE_SP: Watch<[f32; 3]> = Watch::new();
pub static TRUE_RATE_SP: Watch<[f32; 3]> = Watch::new();
pub static SLEW_RATE_SP: Watch<[f32; 3]> = Watch::new();
pub static FF_PRED_GYR: Watch<[f32; 3]> = Watch::new();

// Setpoint outputs, to be routed by the signal_router task
pub static POS_TO_VEL_SP: Watch<[f32; 3]> = Watch::new();
pub static VEL_TO_ANGLE_SP: Watch<[f32; 3]> = Watch::new();
pub static ANGLE_TO_RATE_SP: Watch<[f32; 3]> = Watch::new();
pub static RC_AXES_SP: Watch<[f32; 3]> = Watch::new();

// Data received from an RC controller
pub static RC_CHANNELS_RAW: Watch<Option<[u16; 16]>> = Watch::new();
pub static RC_ANALOG_RATE: Watch<RcAnalog> = Watch::new();
pub static RC_ANALOG_UNIT: Watch<RcAnalog> = Watch::new();
pub static RC_STATUS: Watch<RcStatus> = Watch::new();

pub static RATE_PIPELINE_TIME: Watch<[Duration; 4]> = Watch::new();

//
pub static RATE_PID_TERMS: Watch<[PidTerms; 3]> = Watch::new();
pub static COMP_FUSE_GYR: Watch<[f32; 3]> = Watch::new();

pub static ANGLE_PID_TERMS: Watch<[PidTerms; 3]> = Watch::new();

// Control mode, defined by the commander task
pub static CONTROL_MODE: Watch<ControlMode> = Watch::new();

// Enable or disable the integrators for the attitude controllers
pub static ATTITUDE_INT_EN: Watch<bool> = Watch::new();

/// Arming blocker, defines whether it is safe to arm the vehicle.
pub static ARMING_BLOCKER: Watch<ArmingBlocker> = Watch::new();

pub static MOTORS_STATE: Watch<MotorsState> = Watch::new();

// Motor speeds published by the rate controller
pub static CTRL_MOTORS: Watch<[f32; 4]> = Watch::new();

/// Whether the USB is connected or not
pub static USB_CONNECTED: Watch<bool> = Watch::new();

pub static CALIBRATOR_STATE: Watch<CalibratorState> = Watch::new();

// Commander signals
pub static CMD_ARM_MOTORS: Watch<(bool, bool)> = Watch::new();
pub static CMD_CALIBRATE: Watch<Calibrate> = Watch::new();

/// The boot configuration is only read once at startup
pub static BOOT_CONFIG: OnceLock<BootConfig> = OnceLock::new();

// CONFIGURATION SIGNALS //
pub static CFG_RC_BINDINGS: Watch<RcBindings> = Watch::new();
pub static CFG_MOTOR_DIRS: Watch<[bool; 4]> = Watch::new();
pub static CFG_VEHICLE_INFO: Watch<Vehicle> = Watch::new();
pub static CFG_LOG_PRESET: Watch<LogPreset> = Watch::new();

pub static CFG_RATE_LOOP_PIDS: Watch<RatePidCfg3D> = Watch::new();
pub static CFG_CONTROL_FREQ: Watch<u16> = Watch::new();

// The event bus is mainly meant for aggerating system events
pub static COMMANDER_REQUEST: Channel<M, CmdRequest, 10> = Channel::new();
pub static BLACKBOX_QUEUE: Channel<M, LoggableType, 10> = Channel::new();

pub static OUTPUT_OVERRIDE: AtomicBool = AtomicBool::new(false);
pub static IN_FLIGHT: AtomicBool = AtomicBool::new(false);
pub static CONTROL_FREQUENCY: AtomicU16 = AtomicU16::new(0);
pub static ACTIVE_IMU: AtomicUsize = AtomicUsize::new(0);

#[macro_export]
macro_rules! get_ctrl_freq {
    () => {
        loop {
            use $crate::signals::CONTROL_FREQUENCY;
            use core::sync::atomic::Ordering;
            use embassy_time::Timer;
            let freq = CONTROL_FREQUENCY.load(Ordering::Relaxed);
            if freq != 0 {
                break freq;
            } else {
                Timer::after_millis(10).await;
            }
        }
    };
}

multi_watch!(CFG_MULTI_MAG_ROT, Rotation, NUM_MAG, 2);
multi_watch!(CFG_MULTI_MAG_CAL, Calib3DType, NUM_MAG, 2);

multi_watch!(CFG_MULTI_IMU_ROT, Rotation, NUM_IMU, 2);
multi_watch!(CFG_MULTI_ACC_CAL, Calib3DType, NUM_IMU, 2);
multi_watch!(CFG_MULTI_GYR_CAL, Calib3DType, NUM_IMU, 2);

multi_watch!(RAW_MULTI_IMU_DATA, Imu6DofData<f32>, NUM_IMU, 2);
multi_watch!(CAL_MULTI_IMU_DATA, Imu6DofData<f32>, NUM_IMU, 2);


multi_watch!(RAW_MULTI_MAG_DATA, [f32; 3], NUM_MAG, 2);
multi_watch!(CAL_MULTI_MAG_DATA, [f32; 3], NUM_MAG, 2);

pub static IMU_MODES: Watch<[Mode; NUM_IMU]> = Watch::new();

#[cfg(feature = "mavlink")]
pub use crate::mavlink::MAV_REQUEST;
