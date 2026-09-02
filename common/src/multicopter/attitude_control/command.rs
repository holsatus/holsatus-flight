use nalgebra::{UnitQuaternion, Vector3};

use crate::sync::watch::Watch;

/// The setpoint command handed to the attitude controller.
///
/// The attitude controller derives its control law purely from the
/// last-seen variant of this command. It neither knows nor cares about
/// which flight mode produced it. This is the single vocabulary through
/// which flight modes talk to the controller.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AttitudeCommand {
    /// Disengage all attitude control.
    Disengage,

    /// Commanded body angular rates in radians per second.
    Rate(Vector3<f32>),

    /// Commanded body attitude.
    Angle(UnitQuaternion<f32>),
}

/// The latest attitude command, produced by the active flight mode.
pub static ATTITUDE_COMMAND: Watch<AttitudeCommand> = Watch::new();
