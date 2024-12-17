use crate::errors::CalibrationError;

use super::{sens3d::Calib3DType, MagCalib};

pub async fn calibrate_mag(_config: MagCalib) -> Result<Calib3DType, CalibrationError> {
    Err(CalibrationError::MagBadFit)
}
