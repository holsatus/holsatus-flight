use core::ops::Mul;

use nalgebra::{Matrix3, Vector3};
use num_traits::Float;
use crate::constants::INV_SQRT_2;

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
    let c = INV_SQRT_2;
    Vector3::new(x, c * y - c * z, c * y + c * z)
}

/// Rotate a vector 90 degrees around the x-axis
pub fn rot_x_90(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(x, z, -y)
}

/// Rotate a vector 135 degrees around the x-axis
pub fn rot_x_135(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
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
    let c = INV_SQRT_2;
    Vector3::new(x, -c * y + c * z, -c * y - c * z)
}

/// Rotate a vector 270 degrees around the x-axis
pub fn rot_x_270(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(x, -z, y)
}

/// Rotate a vector 315 degrees around the x-axis
pub fn rot_x_315(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
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
    Vector3::new(x, c * y - s * z, c * y + s * z)
}

// ---------------- //
// Y-axis rotations //
// ---------------- //

/// Rotate a vector 45 degrees around the y-axis
pub fn rot_y_45(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
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
    let c = INV_SQRT_2;
    Vector3::new(-c * x + c * z, y, c * x + c * z)
}

/// Rotate a vector 180 degrees around the y-axis
pub fn rot_y_180(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-x, y, -z)
}

/// Rotate a vector 225 degrees around the y-axis
pub fn rot_y_225(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
    Vector3::new(-c * x - c * z, y, -c * x + c * z)
}

/// Rotate a vector 270 degrees around the y-axis
pub fn rot_y_270(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-z, y, x)
}

/// Rotate a vector 315 degrees around the y-axis
pub fn rot_y_315(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
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
    let c = INV_SQRT_2;
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
    let c = INV_SQRT_2;
    Vector3::new(-c * x - c * y, -c * x + c * y, z)
}

/// Rotate a vector 180 degrees around the z-axis
pub fn rot_z_180(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    Vector3::new(-x, -y, z)
}

/// Rotate a vector 225 degrees around the z-axis
pub fn rot_z_225(vec: Vector3<f32>) -> Vector3<f32> {
    let [x, y, z] = <[f32; 3]>::from(vec);
    let c = INV_SQRT_2;
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
    let c = INV_SQRT_2;
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

#[derive(Debug, Clone, Copy)]
pub enum Rotation {
    Identity,
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
    Custom(Matrix3<f32>)
}

impl Mul<Vector3<f32>> for Rotation {
    type Output = Vector3<f32>;

    fn mul(self, rhs: Vector3<f32>) -> Self::Output {
        match self {
            Rotation::Identity => rhs,
            Rotation::RotX45 => rot_x_45(rhs),
            Rotation::RotX90 => rot_x_90(rhs),
            Rotation::RotX135 => rot_x_135(rhs),
            Rotation::RotX180 => rot_x_180(rhs),
            Rotation::RotX225 => rot_x_225(rhs),
            Rotation::RotX270 => rot_x_270(rhs),
            Rotation::RotX315 => rot_x_315(rhs),
            Rotation::RotXAny(x) => rot_x_any(rhs, x),
            Rotation::RotY45 => rot_y_45(rhs),
            Rotation::RotY90 => rot_y_90(rhs),
            Rotation::RotY135 => rot_y_135(rhs),
            Rotation::RotY180 => rot_y_180(rhs),
            Rotation::RotY225 => rot_y_225(rhs),
            Rotation::RotY270 => rot_y_270(rhs),
            Rotation::RotY315 => rot_y_315(rhs),
            Rotation::RotYAny(x) => rot_y_any(rhs, x),
            Rotation::RotZ45 => rot_z_45(rhs),
            Rotation::RotZ90 => rot_z_90(rhs),
            Rotation::RotZ135 => rot_z_135(rhs),
            Rotation::RotZ180 => rot_z_180(rhs),
            Rotation::RotZ225 => rot_z_225(rhs),
            Rotation::RotZ270 => rot_z_270(rhs),
            Rotation::RotZ315 => rot_z_315(rhs),
            Rotation::RotZAny(x) => rot_z_any(rhs, x),
            Rotation::Custom(m) => m*rhs,
        }
    }
}