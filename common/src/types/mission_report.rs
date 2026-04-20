use serde::{Deserialize, Serialize};

/// Summary statistics for a completed mission.
///
/// On real hardware, fields are populated from ESKF estimates and IMU data.
/// In simulation, they may be populated from simulator ground truth instead.
/// Field-level comments note when the source differs.
#[derive(Clone, Serialize, Deserialize)]
pub struct MissionReport {
    /// Whether the mission completed without a ground collision
    pub mission_completed: bool,
    /// Maximum altitude above the starting position [m]
    pub max_altitude_m: f32,
    /// Average ground speed over the full mission [m/s]
    pub avg_speed_ms: f32,
    /// Maximum ground speed [m/s]
    pub max_speed_ms: f32,
    /// Maximum specific force (gravity-subtracted) while airborne [m/s^2]
    pub max_acceleration_ms2: f32,
    /// Whether the vehicle descended below the starting plane
    pub ground_collision: bool,
    /// Speed at last ground contact during landing [m/s]
    pub landing_speed_ms: f32,
    /// Final position in NED frame [m]
    pub final_position_m: [f32; 3],
}
