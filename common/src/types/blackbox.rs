use embassy_time::Instant;
use serde::{Deserialize, Serialize};

use crate::{filters::rate_pid::RatePidCfg3D, rc_mapping::digital::RcEvent, signals::BLACKBOX_QUEUE};

use crate::errors::HolsatusError;
use super::status::PidTerms;

#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Destination(u8);

bitflags::bitflags! {
    impl Destination: u8 {
        const FatStore  = 1 << 0;
        const Flash     = 1 << 1;
        const Usart     = 1 << 2;
        const MAVLink   = 1 << 3;
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RateDivision(u8);

bitflags::bitflags! {
    impl RateDivision: u8 {
        const Fast      = 0b0000_0000;
        const Div2      = 0b0000_0001;
        const Div4      = 0b0000_0011;
        const Div8      = 0b0000_0111;
        const Div16     = 0b0000_1111;
        const Div32     = 0b0001_1111;
        const Div64     = 0b0011_1111;
        const Div128    = 0b0111_1111;
        const Div256    = 0b1111_1111;
        const Disabled  = 0b1010_1010;
    }
}

impl RateDivision {
    fn offset(&self) -> u8 {
        match self {
            &RateDivision::Fast => 0,
            &RateDivision::Div2 => 1,
            &RateDivision::Div4 => 2,
            &RateDivision::Div8 => 3,
            &RateDivision::Div16 => 4,
            &RateDivision::Div32 => 5,
            &RateDivision::Div64 => 6,
            &RateDivision::Div128 => 7,
            &RateDivision::Div256 => 8,
            _ => 0, // Should not happen
        }
    }

    /// Returns true if the subdivision should run for the given counter value.
    ///
    /// This evenly distribute log subdivions to ensure that log data does not
    /// clog the log channel. Compared to `should_run_synced` which will try to
    /// ensure that the subdivisions grouped together in time.
    pub fn should_run_balanced(&self, counter: u8) -> bool {
        self != &RateDivision::Disabled &&
        counter & self.bits() == self.offset()
    }

    /// Returns true if the subdivision should run for the given counter value.
    ///
    /// This will try to group subdivisions together to ensure that log data is
    /// well aligned in time. Compared to `should_run_balanced` which will try
    /// to ensure that the subdivisions are evenly distributed in time.
    pub fn should_run_synced(&self, counter: u8) -> bool {
        self != &RateDivision::Disabled &&
        counter & self.bits() == 0
    }
}


#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LogPreset {
    /// The base log rate (Hz )defines the fastest available logging frequency
    /// in the system. Slower logging rates will be based
    pub base_log_rate: u16,

    /// The rate preset includes raw and calibrated gyroscope and accelerometer
    /// measurements, setpoints, controller outputs and motor speeds.
    pub rate_div: RateDivision,

    /// The angle preset includes the estimated attitude, as well as angle
    /// controller setpoints and outputs.
    pub angle_div: RateDivision,

    /// The velocity preset includes the estimated velocity in NED world
    /// coordinates, as well as velocity controller setpoints and controller
    /// outputs.
    pub velocity_div: RateDivision,

    /// The position preset includes the estimated position in GNSS coordinates,
    /// as well as position controller setpoints and outputs.
    pub position_div: RateDivision,

    /// The mission preset includes current mission status and progress,
    /// estimated arrival times, path smoothing and predictions.
    pub mission_div: RateDivision,

    /// The events preset includes various systems events, such as
    /// arming/disarming flight/landing detection, warnings, errors mode
    /// changes, etc.
    pub events: bool,
}

impl Default for LogPreset {
    fn default() -> Self {
        LogPreset {
            base_log_rate: 1000,
            rate_div: RateDivision::Fast,
            angle_div: RateDivision::Div8,
            velocity_div: RateDivision::Div32,
            position_div: RateDivision::Div32,
            mission_div: RateDivision::Div256,
            events: true,
        }
    }
}

#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BlackboxConfig {
    pub preset: LogPreset,
    pub destination: Destination,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct RateLog {
    timestamp_us: u64,
    setpoint: [f32; 3],
    measurement: [f32; 3],
    pid_int: [PidTerms; 3],
    motors: [u16; 4],
}

pub fn get_rate_log() -> Option<RateLog> {
    use crate::signals as s;
    Some(RateLog {
        timestamp_us: Instant::now().as_micros(),
        setpoint: s::TRUE_ANGLE_SP.try_get()?,
        measurement: s::CAL_IMU_DATA.try_get()?.gyr,
        pid_int: s::RATE_PID_TERMS.try_get()?,
        motors: s::MOTORS_STATE.try_get()?.as_speeds(),
    })
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct RateMetaLog {
    meta: RatePidCfg3D,
}

pub fn get_rate_meta_log() -> Option<RateMetaLog> {
    use crate::signals as s;
    Some(RateMetaLog {
        meta: s::CFG_RATE_LOOP_PIDS.try_get()?,
    })
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct AngleLog {
    timestamp_us: u64,
    setpoint: [f32; 3],
    att_estimate: [f32; 3],
    accelerometer: [f32; 3],
    pid_int: [PidTerms; 3],
}

pub fn get_angle_log() -> Option<AngleLog> {
    use crate::signals as s;
    Some(AngleLog {
        timestamp_us: Instant::now().as_micros(),
        setpoint: s::TRUE_ANGLE_SP.try_get()?,
        att_estimate: s::AHRS_ATTITUDE.try_get()?,
        accelerometer: s::CAL_IMU_DATA.try_get()?.acc,
        pid_int: s::ANGLE_PID_TERMS.try_get()?,
    })
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct VelocityLog {
    timestamp_us: u64,
    setpoint: [f32; 3],
    estimate: [f32; 3],
    pid_int: [PidTerms; 3],
    gnss_vel: f32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct PositionLog {
    timestamp_us: u64,
    setpoint: [f32; 3],
    estimate: [f32; 3],
    lat_lon: [i32; 2],
    altitude: f32,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct MissionLog {
    timestamp_us: u64,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum Event {
    RcEvent(RcEvent),
    Error(HolsatusError),
    // ArmBlocker(ArmingBlocker), enable serde
}

impl From<HolsatusError> for LoggableType {
    fn from(value: HolsatusError) -> Self {
        LoggableType::Event(
            TsEvent {
                timestamp_us: Instant::now().as_micros(),
                event: Event::Error(value),
            }
        )
    }
}

impl From<RcEvent> for LoggableType {
    fn from(value: RcEvent) -> Self {
        LoggableType::Event(
            TsEvent {
                timestamp_us: Instant::now().as_micros(),
                event: Event::RcEvent(value),
            }
        )
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct TsEvent {
    pub timestamp_us: u64,
    pub event: Event,
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum LoggableType {
    RateMeta(RateMetaLog),
    Rate(RateLog),
    Angle(AngleLog),
    Velocity(VelocityLog),
    Position(PositionLog),
    Misson(MissionLog),
    Event(TsEvent),
}

/// Log an event to the blackbox
pub async fn log_event(event: Event) {
    BLACKBOX_QUEUE
        .send(LoggableType::Event(TsEvent {
            timestamp_us: Instant::now().as_micros(),
            event,
        }
    )).await
}

/// Send an error to be logged in the blackbox
pub fn log_error(error: impl Into<HolsatusError>) {
    // TODO: Handle full queue better TODO: Add option to send to telemetry
    _ = BLACKBOX_QUEUE
        .try_send(LoggableType::Event(TsEvent {
            timestamp_us: Instant::now().as_micros(),
            event: Event::Error(error.into()),
        }
    ));
}
