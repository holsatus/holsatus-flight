use embassy_time::Duration;

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
