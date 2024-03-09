pub mod icm20948;
pub mod types;

use crate::config::{Calibration, Extrinsics};
use nalgebra::Vector3;

pub fn apply_config(
    ext: Option<&Extrinsics>,
    cal: Option<&Calibration>,
    vec: &mut Vector3<f32>,
) {
    // Apply valid calibration to all measurements
    if let Some(mag_cal) = cal {
        mag_cal.apply(vec)
    }

    // Apply extrinsic configurations
    if let Some(mag_ext) = ext {
        mag_ext.apply(vec)
    }
}
