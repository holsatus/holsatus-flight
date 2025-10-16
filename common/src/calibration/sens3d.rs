use nalgebra::Vector3;

/// Calibration data for a "small" calibration, meaning it only has bias and scale.
#[derive(Debug, Copy, Clone, PartialEq, mav_param::Tree)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Calib3D {
    pub bias: [f32; 3],
    #[param(rename = "scal")]
    pub scale: [f32; 3],
}

impl Calib3D {
    pub const fn const_default() -> Self {
        Self {
            bias: [0.; 3],
            scale: [1.; 3],
        }
    }
}

impl Calib3D {
    pub fn apply_mut(&self, data: &mut Vector3<f32>) {
        for i in 0..3 {
            data[i] -= self.bias[i];
            data[i] *= self.scale[i];
        }
    }

    pub fn apply(&self, mut data: Vector3<f32>) -> Vector3<f32> {
        self.apply_mut(&mut data);
        data
    }
}
