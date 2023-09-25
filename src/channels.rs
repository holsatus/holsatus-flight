/*

    The communication channel type commonly used on this firmware is the Embassy PubSubChannel.

    This module defines the channel types which interconnect all Embassy tasks, and act as the only
    method for tasks to communicate values and states. By using the PubSubChannel type with a single
    publisher and a capacity of 1, the channel will work effectively like a shared state, which 
    subscribers can access at will, by either awaiting a change in the value, or simply by checking.

*/


use embassy_sync::{
    pubsub::{PubSubChannel,Publisher,Subscriber},
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex}
};
use nalgebra::Vector3;

// Take a value from a channel if it exists and assign it to the existing variable, returning true if it existed
pub fn update_from_channel<const CAP : usize, const SUBS : usize, const PUBS : usize, M:RawMutex, T:Clone>
(channel : &mut Subscriber<M,T,CAP,SUBS,PUBS>, variable : &mut T) -> bool {
    match channel.try_next_message_pure() {
        Some(new_value) => {
            *variable = new_value;
            true
        },
        None => false
    }
}

type ChannelMutex = CriticalSectionRawMutex;

// Short-hand type alias for PubSubChannel
pub type Pub<T,const N: usize> = Publisher<'static,ChannelMutex,T,1,N,1>;
pub type Sub<T,const N: usize> = Subscriber<'static,ChannelMutex,T,1,N,1>;
pub type Ch<T,const N: usize> = PubSubChannel<ChannelMutex,T,1,N,1>;

const IMU_READING_NUM: usize = 2;
pub type ImuReadingType = crate::imu::Dof6ImuData<f32>;
pub type ImuReadingPub = Pub<ImuReadingType,IMU_READING_NUM>;
pub type ImuReadingSub = Sub<ImuReadingType,IMU_READING_NUM>;
pub static IMU_READING : Ch<ImuReadingType,IMU_READING_NUM> = PubSubChannel::new();

const ATTITUDE_SENSE_NUM: usize = 3;
pub type AttitudeSenseType = (Vector3<f32>,Vector3<f32>);
pub type AttitudeSensePub = Pub<AttitudeSenseType,ATTITUDE_SENSE_NUM>;
pub type AttitudeSenseSub = Sub<AttitudeSenseType,ATTITUDE_SENSE_NUM>;
pub static ATTITUDE_SENSE : Ch<AttitudeSenseType,ATTITUDE_SENSE_NUM> = PubSubChannel::new();

const ATTITUDE_ACTUATE_NUM: usize = 2;
pub type AttitudeActuateType = Vector3<f32>;
pub type AttitudeActuatePub = Pub<AttitudeActuateType,ATTITUDE_ACTUATE_NUM>;
pub type AttitudeActuateSub = Sub<AttitudeActuateType,ATTITUDE_ACTUATE_NUM>;
pub static ATTITUDE_ACTUATE : Ch<AttitudeActuateType,ATTITUDE_ACTUATE_NUM> = PubSubChannel::new();

const ATTITUDE_INT_ENABLE_NUM: usize = 1;
pub type AttitudeIntEnableType = bool;
pub type AttitudeIntEnablePub = Pub<AttitudeIntEnableType,ATTITUDE_INT_ENABLE_NUM>;
pub type AttitudeIntEnableSub = Sub<AttitudeIntEnableType,ATTITUDE_INT_ENABLE_NUM>;
pub static ATTITUDE_INT_ENABLE : Ch<AttitudeIntEnableType,ATTITUDE_INT_ENABLE_NUM> = PubSubChannel::new();

const ATTITUDE_STAB_MODE_NUM: usize = 2;
pub type AttitudeStabModeType = crate::task_attitude_controller::StabilizationMode;
pub type AttitudeStabModePub = Pub<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM>;
pub type AttitudeStabModeSub = Sub<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM>;
pub static ATTITUDE_STAB_MODE : Ch<AttitudeStabModeType,ATTITUDE_STAB_MODE_NUM> = PubSubChannel::new();

const MOTOR_SPEED_NUM: usize = 1;
pub type MotorSpeedType = Option<[u16;4]>;
pub type MotorSpeedPub = Pub<MotorSpeedType,MOTOR_SPEED_NUM>;
pub type MotorSpeedSub = Sub<MotorSpeedType,MOTOR_SPEED_NUM>;
pub static MOTOR_SPEED : Ch<MotorSpeedType,MOTOR_SPEED_NUM> = PubSubChannel::new();

const MOTOR_ARM_NUM: usize = 1;
pub type MotorArmType = bool;
pub type MotorArmPub = Pub<MotorArmType,MOTOR_ARM_NUM>;
pub type MotorArmSub = Sub<MotorArmType,MOTOR_ARM_NUM>;
pub static MOTOR_ARM : Ch<MotorArmType,MOTOR_ARM_NUM> = PubSubChannel::new();

const MOTOR_SPIN_CHECK_NUM: usize = 1;
pub type MotorSpinCheckType = bool;
pub type MotorSpinCheckPub = Pub<MotorSpinCheckType,MOTOR_SPIN_CHECK_NUM>;
pub type MotorSpinCheckSub = Sub<MotorSpinCheckType,MOTOR_SPIN_CHECK_NUM>;
pub static MOTOR_SPIN_CHECK : Ch<MotorSpinCheckType,MOTOR_SPIN_CHECK_NUM> = PubSubChannel::new();

const MOTOR_DIR_NUM: usize = 1;
pub type MotorDirType = (bool,bool,bool,bool);
pub type MotorDirPub = Pub<MotorDirType,MOTOR_DIR_NUM>;
pub type MotorDirSub = Sub<MotorDirType,MOTOR_DIR_NUM>;
pub static MOTOR_DIR : Ch<MotorDirType,MOTOR_DIR_NUM> = PubSubChannel::new();

const MOTOR_STATE_NUM: usize = 3;
pub type MotorStateType = crate::task_motor_governor::MotorState;
pub type MotorStatePub = Pub<MotorStateType,MOTOR_STATE_NUM>;
pub type MotorStateSub = Sub<MotorStateType,MOTOR_STATE_NUM>;
pub static MOTOR_STATE : Ch<MotorStateType,MOTOR_STATE_NUM> = PubSubChannel::new();

const SBUS_CMD_NUM: usize = 1;
pub type SbusCmdType = Result<crate::sbus_cmd::SbusCmd,crate::task_sbus_reader::SbusError>;
pub type SbusCmdPub = Pub<SbusCmdType,SBUS_CMD_NUM>;
pub type SbusCmdSub = Sub<SbusCmdType,SBUS_CMD_NUM>;
pub static SBUS_CMD : Ch<SbusCmdType,SBUS_CMD_NUM> = PubSubChannel::new();

const THRUST_ACTUATE_NUM: usize = 2;
pub type ThrustActuateType = f32;
pub type ThrustActuatePub = Pub<ThrustActuateType,THRUST_ACTUATE_NUM>;
pub type ThrustActuateSub = Sub<ThrustActuateType,THRUST_ACTUATE_NUM>;
pub static THRUST_ACTUATE : Ch<ThrustActuateType,THRUST_ACTUATE_NUM> = PubSubChannel::new();

const BLINKER_MODE_NUM: usize = 1;
pub type BlinkerModeType = crate::task_blinker::BlinkerMode;
pub type BlinkerModePub = Pub<BlinkerModeType,BLINKER_MODE_NUM>;
pub type BlinkerModeSub = Sub<BlinkerModeType,BLINKER_MODE_NUM>;
pub static BLINKER_MODE : Ch<BlinkerModeType,BLINKER_MODE_NUM> = PubSubChannel::new();

const FLIGHT_MODE_NUM: usize = 2;
pub type FlightModeType = crate::task_flight_detector::FlightMode;
pub type FlightModePub = Pub<FlightModeType,FLIGHT_MODE_NUM>;
pub type FlightModeSub = Sub<FlightModeType,FLIGHT_MODE_NUM>;
pub static FLIGHT_MODE : Ch<FlightModeType,FLIGHT_MODE_NUM> = PubSubChannel::new();

/* DO NOT MODIFY PROTOTYPE
const CH_PROTOTYPE_X_NUM: usize = 1;
pub type ChPrototypeXType = bool;
pub type ChPrototypeXPub = Pub<ChPrototypeXType,CH_PROTOTYPE_X_NUM>;
pub type ChPrototypeXSub = Sub<ChPrototypeXType,CH_PROTOTYPE_X_NUM>;
pub static CH_PROTOTYPE_X : Ch<ChPrototypeXType,CH_PROTOTYPE_X_NUM> = PubSubChannel::new();
*/

pub fn assert_all_subscribers_used() {
    assert!(IMU_READING.subscriber().is_err());
    assert!(ATTITUDE_SENSE.subscriber().is_err());
    assert!(ATTITUDE_ACTUATE.subscriber().is_err());
    assert!(ATTITUDE_INT_ENABLE.subscriber().is_err());
    assert!(ATTITUDE_STAB_MODE.subscriber().is_err());
    assert!(MOTOR_SPEED.subscriber().is_err());
    assert!(MOTOR_ARM.subscriber().is_err());
    assert!(MOTOR_SPIN_CHECK.subscriber().is_err());
    assert!(MOTOR_DIR.subscriber().is_err());
    assert!(MOTOR_STATE.subscriber().is_err());
    assert!(SBUS_CMD.subscriber().is_err());
    assert!(THRUST_ACTUATE.subscriber().is_err());
    assert!(BLINKER_MODE.subscriber().is_err());
    assert!(FLIGHT_MODE.subscriber().is_err());
}