use core::ops::Mul;

#[allow(unused_imports)]
use num_traits::Float;
use serde::{Deserialize, Serialize};

use core::f32::consts::FRAC_1_SQRT_2;
use nalgebra::{Matrix3, Vector3};

// For reasons of efficiency, most of the rotation functions are implemented not
// as matrix-vector multiplications, but instead as simple vector manipulations.
// This is because the rotation matrices are mostly composed of 1s, -1s and 0s.

pub fn rot_identity(vec: Vector3<f32>) -> Vector3<f32> {
    vec
}

// ---------------- //
// X-axis rotations //
// ---------------- //

/// Rotate a vector 45 degrees around the x-axis
pub fn rot_x_45(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(x, c * y - c * z, c * y + c * z)
}

/// Rotate a vector 90 degrees around the x-axis
pub fn rot_x_90(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(x, -z, y)
}

/// Rotate a vector 135 degrees around the x-axis
pub fn rot_x_135(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(x, -c * y - c * z, c * y - c * z)
}

/// Rotate a vector 180 degrees around the x-axis
pub fn rot_x_180(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(x, -y, -z)
}

/// Rotate a vector 225 degrees around the x-axis
pub fn rot_x_225(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(x, -c * y + c * z, -c * y - c * z)
}

/// Rotate a vector 270 degrees around the x-axis
pub fn rot_x_270(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(x, z, -y)
}

/// Rotate a vector 315 degrees around the x-axis
pub fn rot_x_315(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(x, c * y + c * z, -c * y + c * z)
}

/// Rotate a vector by an arbitrary angle around the x-axis
///
/// **Note**: If possible use the pre-defined `rot_x_{45, 90, 135, 180, 225, 270, 315}` functions.
pub fn rot_x_any(vec: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let angle = angle.to_radians();
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = angle.cos();
    let s = angle.sin();
    Vector3::new(x, c * y - s * z, s * y + c * z)
}

// ---------------- //
// Y-axis rotations //
// ---------------- //

/// Rotate a vector 45 degrees around the y-axis
pub fn rot_y_45(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(c * x + c * z, y, -c * x + c * z)
}

/// Rotate a vector 90 degrees around the y-axis
pub fn rot_y_90(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(z, y, -x)
}

/// Rotate a vector 135 degrees around the y-axis
pub fn rot_y_135(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(-c * x + c * z, y, -c * x - c * z)
}

/// Rotate a vector 180 degrees around the y-axis
pub fn rot_y_180(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-x, y, -z)
}

/// Rotate a vector 225 degrees around the y-axis
pub fn rot_y_225(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(-c * x - c * z, y, c * x - c * z)
}

/// Rotate a vector 270 degrees around the y-axis
pub fn rot_y_270(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-z, y, x)
}

/// Rotate a vector 315 degrees around the y-axis
pub fn rot_y_315(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(c * x - c * z, y, c * x + c * z)
}

/// Rotate a vector by an arbitrary angle around the y-axis
///
/// **Note**: If possible use the pre-defined `rot_y_{45, 90, 135, 180, 225, 270, 315}` functions.
pub fn rot_y_any(vec: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let angle = angle.to_radians();
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = angle.cos();
    let s = angle.sin();
    Vector3::new(c * x + s * z, y, -s * x + c * z)
}

// ---------------- //
// Z-axis rotations //
// ---------------- //

/// Rotate a vector 45 degrees around the z-axis
pub fn rot_z_45(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(c * x - c * y, c * x + c * y, z)
}

/// Rotate a vector 90 degrees around the z-axis
pub fn rot_z_90(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-y, x, z)
}

/// Rotate a vector 135 degrees around the z-axis
pub fn rot_z_135(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(-c * x - c * y, c * x - c * y, z)
}

/// Rotate a vector 180 degrees around the z-axis
pub fn rot_z_180(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-x, -y, z)
}

/// Rotate a vector 225 degrees around the z-axis
pub fn rot_z_225(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(-c * x + c * y, -c * x - c * y, z)
}

/// Rotate a vector 270 degrees around the z-axis
pub fn rot_z_270(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(y, -x, z)
}

/// Rotate a vector 315 degrees around the z-axis
pub fn rot_z_315(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = FRAC_1_SQRT_2;
    Vector3::new(c * x + c * y, -c * x + c * y, z)
}

/// Rotate a vector by an arbitrary angle around the z-axis
///
/// **Note**: If possible use the pre-defined `rot_z_{45, 90, 135, 180, 225, 270, 315}` functions.
pub fn rot_z_any(vec: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let angle = angle.to_radians();
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = angle.cos();
    let s = angle.sin();
    Vector3::new(c * x - s * y, s * x + c * y, z)
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, mav_param::Enum)]
#[repr(u8)]
pub enum Rotation {
    #[default]
    Identity = 0,
    RotX45,
    RotX90,
    RotX135,
    RotX180,
    RotX225,
    RotX270,
    RotX315,
    RotXAny(f32),
    RotY45,
    RotY90,
    RotY135,
    RotY180,
    RotY225,
    RotY270,
    RotY315,
    RotYAny(f32),
    RotZ45,
    RotZ90,
    RotZ135,
    RotZ180,
    RotZ225,
    RotZ270,
    RotZ315,
    RotZAny(f32),
    Custom([[f32; 3]; 3]),
}

impl Rotation {
    pub const fn const_default() -> Self {
        Rotation::Identity
    }
}

impl Rotation {
    /// Construct a rotation matrix from the given angles in degrees
    pub fn custom(x: f32, y: f32, z: f32) -> Self {
        assert!(x >= 0.0 && x < 360.0);
        assert!(y >= 0.0 && y < 360.0);
        assert!(z >= 0.0 && z < 360.0);

        let (xr, yr, zr) = (x.to_radians(), y.to_radians(), z.to_radians());
        let (sin_x, cos_x) = (xr.sin(), xr.cos());
        let (sin_y, cos_y) = (yr.sin(), yr.cos());
        let (sin_z, cos_z) = (zr.sin(), zr.cos());

        // Rotation matrix around the X-axis
        let rot_x = Matrix3::new(1.0, 0.0, 0.0, 0.0, cos_x, -sin_x, 0.0, sin_x, cos_x);

        // Rotation matrix around the Y-axis
        let rot_y = Matrix3::new(cos_y, 0.0, sin_y, 0.0, 1.0, 0.0, -sin_y, 0.0, cos_y);

        // Rotation matrix around the Z-axis
        let rot_z = Matrix3::new(cos_z, -sin_z, 0.0, sin_z, cos_z, 0.0, 0.0, 0.0, 1.0);

        let rotation = rot_z * rot_y * rot_x;

        // Combined rotation matrix
        Rotation::Custom(rotation.data.0)
    }
}

impl Mul<Vector3<f32>> for &Rotation {
    type Output = Vector3<f32>;

    fn mul(self, rhs: Vector3<f32>) -> Self::Output {
        use Rotation as R;
        match self {
            R::Identity => rhs,
            R::RotX45 => rot_x_45(rhs),
            R::RotX90 => rot_x_90(rhs),
            R::RotX135 => rot_x_135(rhs),
            R::RotX180 => rot_x_180(rhs),
            R::RotX225 => rot_x_225(rhs),
            R::RotX270 => rot_x_270(rhs),
            R::RotX315 => rot_x_315(rhs),
            R::RotXAny(x) => rot_x_any(rhs, *x),
            R::RotY45 => rot_y_45(rhs),
            R::RotY90 => rot_y_90(rhs),
            R::RotY135 => rot_y_135(rhs),
            R::RotY180 => rot_y_180(rhs),
            R::RotY225 => rot_y_225(rhs),
            R::RotY270 => rot_y_270(rhs),
            R::RotY315 => rot_y_315(rhs),
            R::RotYAny(x) => rot_y_any(rhs, *x),
            R::RotZ45 => rot_z_45(rhs),
            R::RotZ90 => rot_z_90(rhs),
            R::RotZ135 => rot_z_135(rhs),
            R::RotZ180 => rot_z_180(rhs),
            R::RotZ225 => rot_z_225(rhs),
            R::RotZ270 => rot_z_270(rhs),
            R::RotZ315 => rot_z_315(rhs),
            R::RotZAny(x) => rot_z_any(rhs, *x),
            R::Custom(m) => {
                let storage = nalgebra::ArrayStorage(*m);
                let matrix = nalgebra::Matrix::from_array_storage(storage);
                matrix * rhs
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::abs_diff_eq;

    fn rot_mat(x: f32, y: f32, z: f32) -> Matrix3<f32> {
        let (sin_x, cos_x) = x.to_radians().sin_cos();
        let (sin_y, cos_y) = y.to_radians().sin_cos();
        let (sin_z, cos_z) = z.to_radians().sin_cos();

        // Rotation matrix around the X-axis
        let rot_x = Matrix3::new(1.0, 0.0, 0.0, 0.0, cos_x, -sin_x, 0.0, sin_x, cos_x);

        // Rotation matrix around the Y-axis
        let rot_y = Matrix3::new(cos_y, 0.0, sin_y, 0.0, 1.0, 0.0, -sin_y, 0.0, cos_y);

        // Rotation matrix around the Z-axis
        let rot_z = Matrix3::new(cos_z, -sin_z, 0.0, sin_z, cos_z, 0.0, 0.0, 0.0, 1.0);

        // Combined rotation matrix
        rot_z * rot_y * rot_x
    }

    static AXES: [Vector3<f32>; 3] = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
    ];

    #[test]
    fn test_rot_x() {
        let rotations = [
            rot_x_45, rot_x_90, rot_x_135, rot_x_180, rot_x_225, rot_x_270, rot_x_315,
        ];

        for (i, rot) in rotations.iter().enumerate() {
            for axis in AXES.iter() {
                let angle = (i + 1) as f32 * 45.0;
                let expected = rot_mat(angle, 0., 0.);

                if !abs_diff_eq!(rot(*axis), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nGot: {:?}\nExpected: {:?}",
                        angle,
                        rot(*axis).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }

    #[test]
    fn test_rot_x_any() {
        let angles = [10., 20., 30., 300., 500., 700.];
        for angle in angles {
            for axis in AXES.iter() {
                let expected = rot_mat(angle, 0., 0.);

                if !abs_diff_eq!(rot_x_any(*axis, angle), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nFrom: {:?}\nGot: {:?}\nExpected: {:?}",
                        angle,
                        (*axis).as_slice(),
                        rot_x_any(*axis, angle).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }

    #[test]
    fn test_rot_y() {
        let rotations = [
            rot_y_45, rot_y_90, rot_y_135, rot_y_180, rot_y_225, rot_y_270, rot_y_315,
        ];

        for (i, rot) in rotations.iter().enumerate() {
            for axis in AXES.iter() {
                let angle = (i + 1) as f32 * 45.0;
                let expected = rot_mat(0., angle, 0.);

                if !abs_diff_eq!(rot(*axis), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nFrom {:?}:\nGot: {:?}\nExpected: {:?}",
                        angle,
                        (*axis).as_slice(),
                        (rot(*axis)).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }

    #[test]
    fn test_rot_y_any() {
        let angles = [10., 20., 30., 300., 500., 700.];
        for angle in angles {
            for axis in AXES.iter() {
                let expected = rot_mat(0., angle, 0.);

                if !abs_diff_eq!(rot_y_any(*axis, angle), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nFrom: {:?}\nGot: {:?}\nExpected: {:?}",
                        angle,
                        (*axis).as_slice(),
                        rot_y_any(*axis, angle).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }

    #[test]
    fn test_rot_z() {
        let rotations = [
            rot_z_45, rot_z_90, rot_z_135, rot_z_180, rot_z_225, rot_z_270, rot_z_315,
        ];

        for (i, rot) in rotations.iter().enumerate() {
            for axis in AXES.iter() {
                let angle = (i + 1) as f32 * 45.0;
                let expected = rot_mat(0., 0., angle);

                if !abs_diff_eq!(rot(*axis), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nFrom {:?}:\nGot: {:?}\nExpected: {:?}",
                        angle,
                        (*axis).as_slice(),
                        (rot(*axis)).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }

    #[test]
    fn test_rot_z_any() {
        let angles = [10., 20., 30., 300., 500., 700.];
        for angle in angles {
            for axis in AXES.iter() {
                let expected = rot_mat(0., 0., angle);

                if !abs_diff_eq!(rot_z_any(*axis, angle), expected * *axis) {
                    panic!(
                        "Failed for angle: {}\nFrom: {:?}\nGot: {:?}\nExpected: {:?}",
                        angle,
                        (*axis).as_slice(),
                        rot_z_any(*axis, angle).as_slice(),
                        (expected * *axis).as_slice()
                    );
                }
            }
        }
    }
}
