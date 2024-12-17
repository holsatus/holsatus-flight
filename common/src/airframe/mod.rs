use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum QuadRotorMixing {
    /// Quadcopter "x" configuration
    /// ```text
    ///   front
    /// M4     M2
    ///   \   /
    ///     |
    ///   /   \
    /// M3     M1
    /// ```
    /// where \
    /// `M1` spins CW \
    /// `M2` spins CCW \
    /// `M3` spins CCW \
    /// `M4` spins CW
    QuadX,
    /// Quadcopter "+" configuration
    /// ```text
    ///    front
    ///      M2
    ///      |
    /// M4---+---M1
    ///      |
    ///      M3
    /// ```
    /// where \
    /// `M1` spins CW \
    /// `M2` spins CCW \
    /// `M3` spins CCW \
    /// `M4` spins CW
    QuadP,
}

impl QuadRotorMixing {
    /// Motor mixing functions
    /// - `t` is the thrust
    /// - `a` is the rotation vector
    /// - `rev` reverses motor directions
    pub fn mixing_fn(&self, t: f32, mut a: [f32; 3], rev: bool) -> [f32; 4] {
        if rev {
            a[2] = -a[2];
        }
        match self {
            QuadRotorMixing::QuadX => quad_x_motor_mixing(t, a),
            QuadRotorMixing::QuadP => quad_p_motor_mixing(t, a),
        }
    }
}

#[rustfmt::skip]
pub fn quad_x_motor_mixing(t: f32, a: [f32; 3]) -> [f32; 4] {
    let [x, y, z] = a;
    [
        t - x - y - z,
        t - x + y + z,
        t + x - y + z,
        t + x + y - z
    ]
}

#[rustfmt::skip]
fn quad_p_motor_mixing(t: f32, a: [f32; 3]) -> [f32; 4] {
    let [x, y, z] = a;
    [
        t - x - z,
        t + y + z,
        t - y + z,
        t + x - z
    ]
}
