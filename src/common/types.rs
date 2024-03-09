use defmt::Format;
use nalgebra::Vector3;

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum MotorState {
    Disarmed(DisarmReason),
    Armed(ArmedState),
    Arming,
}

impl MotorState {
    pub fn is_armed(&self) -> bool {
        match self {
            MotorState::Armed(_) => true,
            _ => false,
        }
    }

    pub fn is_disarmed(&self) -> bool {
        match self {
            MotorState::Disarmed(_) => true,
            _ => false,
        }
    }

    pub fn is_arming(&self) -> bool {
        match self {
            MotorState::Arming => true,
            _ => false,
        }
    }
}

impl Default for MotorState {
    fn default() -> Self {
        MotorState::Disarmed(DisarmReason::NotInitialized)
    }
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum ArmedState {
    Running([u16; 4]),
    Idle,
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum DisarmReason {
    NotInitialized,
    Commanded,
    Timeout,
    Fault,
}

#[derive(Clone, Copy, Debug)]
pub enum AttitudeReference {
    Angle(Vector3<f32>),
    Rate(Vector3<f32>),
    None,
}


#[derive(Debug, Clone)]
pub enum VehicleState {
    Uninit,
    Boot,
    Calibrating,
    Standby,
    Active,
    Critical,
    Emergency,
    Poweroff,
    Termination,
}

use mavlink::common::MavState;
impl Into<MavState> for VehicleState {
    fn into(self) -> MavState {
        match self {
            VehicleState::Uninit => MavState::MAV_STATE_ACTIVE,
            VehicleState::Boot => MavState::MAV_STATE_BOOT,
            VehicleState::Calibrating => MavState::MAV_STATE_CALIBRATING,
            VehicleState::Standby => MavState::MAV_STATE_STANDBY,
            VehicleState::Active => MavState::MAV_STATE_ACTIVE,
            VehicleState::Critical => MavState::MAV_STATE_CRITICAL,
            VehicleState::Emergency => MavState::MAV_STATE_EMERGENCY,
            VehicleState::Poweroff => MavState::MAV_STATE_POWEROFF,
            VehicleState::Termination => MavState::MAV_STATE_FLIGHT_TERMINATION,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StabilizationMode {
    Angle,
    Rate,
    None,
}