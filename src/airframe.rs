use nalgebra::{Vector3, Vector4};


#[derive(Debug, Clone, Copy)]
pub enum MotorMixing {
    /// Quadcopter "x" configuration
    /// ```
    ///   front
    /// M3     M1
    ///   \   /
    ///     |
    ///   /   \
    /// M2     M4
    /// ```
    /// where \
    /// `M1` spins CCW \
    /// `M2` spins CW \
    /// `M3` spins CCW \
    /// `M4` spins CW
    QuadX,
    /// Quadcopter "+" configuration
    /// ```
    ///    front
    ///      M1
    ///      |
    /// M3---+---M4
    ///      |
    ///      M2
    /// ```
    /// where \
    /// `M1` spins CCW \
    /// `M2` spins CW \
    /// `M3` spins CCW \
    /// `M4` spins CW
    QuadPlus,
}

impl MotorMixing {
    pub fn mixing_fn(&self, t: f32, a: Vector3<f32>) -> Vector4<f32> {
        match self {
            MotorMixing::QuadX => quad_x_motor_mixing(t, a),
            MotorMixing::QuadPlus => quad_plus_motor_mixing(t, a),
        }
    }
}

fn quad_x_motor_mixing(t: f32, attitude: Vector3<f32>) -> Vector4<f32> {
    let [x, y, z] = attitude.into();
    
    Vector4::new(
        t - x + y + z,
        t + x - y + z,
        t + x + y - z,
        t - x - y - z,
    )
}

fn quad_plus_motor_mixing(t: f32, attitude: Vector3<f32>) -> Vector4<f32> {
    let [x, y, z] = attitude.into();
    
    Vector4::new(
        t + y + z,
        t - y + z,
        t + x - z,
        t - x - z,
    )
}
