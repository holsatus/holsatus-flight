use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SitlState {
    /// Unit Quaternion representing the rotation of the drone
    pub rotation: [f32; 4],
    /// Position of the drone in the world frame
    pub position: [f32; 3],
    /// Velocity of the drone in the world frame
    pub velocity: [f32; 3],
    /// Acceleration of the drone in the body frame
    pub body_acc: [f32; 3],
    /// Angular velocity of the drone in the body frame
    pub body_gyr: [f32; 3],
    /// Force produced by each motor [N]
    pub motors_f: [f32; 4],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SitlCommand {
    /// Flip the drone right side up
    MakeUpright,
    /// Reset the position of the drone to the origin
    ResetPosition,
    /// Set the motor speeds of the drone
    MotorsCmd([u16; 4]),
    /// Advance te simulation by a time step
    SimStep(f32),
}
