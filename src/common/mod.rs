use bitflags::bitflags;

mod mav_bitflag;
pub mod rotation_matrices;
pub mod types;

bitflags! {
    /// High latency telemetry failure flags
    ///
    /// https://mavlink.io/en/messages/common.html#HL_FAILURE_FLAG
    pub struct HlFailureFlag: u16 {
        const GPS               = 0b_00000000000001;
        const DIF_PRESSURE      = 0b_00000000000010;
        const ABS_PRESSURE      = 0b_00000000000100;
        const ACCELEROMETER_3D  = 0b_00000000001000;
        const GYROSCOPE_3D      = 0b_00000000010000;
        const MAGNETOMETER_3D   = 0b_00000000100000;
        const TERRAIN           = 0b_00000001000000;
        const BATTERY           = 0b_00000010000000;
        const RC_RECEIVER       = 0b_00000100000000;
        const OFFBOARD_LINK     = 0b_00001000000000;
        const ENGINE            = 0b_00010000000000;
        const GEOFENCE          = 0b_00100000000000;
        const ESTIMATOR         = 0b_01000000000000;
        const MISSION           = 0b_10000000000000;
    }
}
