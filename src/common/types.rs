use defmt::Format;
use embassy_time::Duration;
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StabilizationMode {
    Angle,
    Rate,
    None,
}

/// Mavlink message types that can be streamed, i.e. can be sent at
/// regular intervals upon request from other Mavlink devices.
pub enum MavStreamable {
    Heartbeat,
    SystemTime,
    Attitude,
    GpsRawInt,
    ScaledImu,
    GlobalPosition,
    RcChannels,
}

#[derive(Clone, Copy, Debug)]
pub struct MavStreamableFrequencies {
    pub heartbeat: Option<Duration>,
    pub system_time: Option<Duration>,
    pub attitude: Option<Duration>,
    pub gps_raw_int: Option<Duration>,
    pub scaled_imu: Option<Duration>,
    pub rc_channels: Option<Duration>,
}

impl MavStreamable {
    pub fn from_id(id: u32) -> Option<MavStreamable> {
        match id {
            0 => Some(MavStreamable::Heartbeat),
            2 => Some(MavStreamable::SystemTime),
            30 => Some(MavStreamable::Attitude),
            24 => Some(MavStreamable::GpsRawInt),
            26 => Some(MavStreamable::ScaledImu),
            33 => Some(MavStreamable::GlobalPosition),
            34 => Some(MavStreamable::RcChannels),
            _ => None,
        }
    }
}
