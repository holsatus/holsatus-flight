use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

/// Wrapper type for different calibration types
#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum Calib3DType {
    None,
    Small(SmallCalib3D),
    Full(FullCalib3D),
}

impl Default for Calib3DType {
    fn default() -> Self {
        Calib3DType::Small(SmallCalib3D::default())
    }
}

impl Calib3DType {
    pub fn set_bias(&mut self, bias: Vector3<f32>) {
        match self {
            Calib3DType::None => {
                *self = Calib3DType::Small(SmallCalib3D::default());
                self.set_bias(bias);
            },
            Calib3DType::Small(cal) => cal.bias = Some(bias),
            Calib3DType::Full(cal) => cal.bias = Some(bias),
        }
    }

    pub fn apply(&self, data: Vector3<f32>) -> Vector3<f32> {
        match self {
            Calib3DType::None => data,
            Calib3DType::Small(cal) => cal.apply(data),
            Calib3DType::Full(cal) => cal.apply(data),
        }
    }
    pub fn has_bias(&self) -> bool {
        match self {
            Calib3DType::None => false,
            Calib3DType::Small(cal) => cal.bias.is_some(),
            Calib3DType::Full(cal) => cal.bias.is_some(),
        }
    }
}

/// Calibration data for a "small" calibration, meaning it only has bias and scale.
#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct SmallCalib3D {
    pub bias: Option<Vector3<f32>>,
    pub scale: Option<Vector3<f32>>,
}

impl SmallCalib3D {
    fn apply(&self, mut data: Vector3<f32>) -> Vector3<f32> {
        if let Some(bias) = self.bias {
            data -= bias;
        }
        if let Some(scale) = self.scale {
            data.component_mul_assign(&scale);
        }
        data
    }
}

/// Calibration data for a "full" calibration, meaning it has bias and warp.
#[derive(Debug, Copy, Clone, Default, Serialize, Deserialize)]
pub struct FullCalib3D {
    pub bias: Option<Vector3<f32>>,
    pub warp: Option<Matrix3<f32>>,
}

impl FullCalib3D {
    fn apply(&self, mut data: Vector3<f32>) -> Vector3<f32> {
        if let Some(bias) = self.bias {
            data -= Vector3::from(bias);
        }
        if let Some(warp) = self.warp {
            data = warp * data;
        }
        data
    }
}
