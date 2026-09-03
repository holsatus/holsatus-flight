use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Used to represent a kind of pseudo-orientation for rate-based control
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeakyQuaternion {
    leaky_gyro: UnitQuaternion<f32>,
    leaky_pred: UnitQuaternion<f32>,
    time_const: f32,
    error_boost: f32,
    dt: f32,
}

impl LeakyQuaternion {
    pub fn new(time_const: f32, error_boost: f32, dt: f32) -> Self {
        Self {
            leaky_gyro: UnitQuaternion::identity(),
            leaky_pred: UnitQuaternion::identity(),
            time_const,
            error_boost,
            dt,
        }
    }

    pub fn reset(&mut self) {
        self.leaky_gyro = UnitQuaternion::identity();
        self.leaky_pred = UnitQuaternion::identity();
    }

    pub fn update(&mut self, gyr_meas: [f32; 3], gyr_pred: [f32; 3], dt: f32) -> [f32; 3] {
        // Compute delta for gyroscope and integrate to yield quaternion
        let gyro_delta = *self.leaky_gyro * Quaternion::from_parts(0.0, gyr_meas.into()) * 0.5;
        self.leaky_gyro = UnitQuaternion::new_unchecked(*self.leaky_gyro + gyro_delta * dt);

        // Compute delta for prediction and integrate to yield quaternion
        let pred_delta = *self.leaky_pred * Quaternion::from_parts(0.0, gyr_pred.into()) * 0.5;
        self.leaky_pred = UnitQuaternion::new_unchecked(*self.leaky_pred + pred_delta * dt);

        // Using the "quaternion error" rather than the euler angle error gives some
        // much nicer behavior where euler angles would normally experiece gimbal lock.
        let q_error = self.leaky_gyro.inverse() * self.leaky_pred;
        let axis_error = approximate_scaled_axis(&q_error);

        // Use error-boosting leak rate if available. Leaks quicker for large errors.
        let alpha = if self.error_boost > 0.0 {
            let time_const = self.time_const / (1.0 + axis_error.norm_squared() * self.error_boost);
            time_const / (time_const + dt)
        } else {
            self.time_const / (self.time_const + dt)
        };

        // Same as the regular `nlerp` function in nalgebra, except this is guaranteed to
        // use the quaternion with the quaternion with the shortest path to identity.
        let nlerp_to_identity = |q: UnitQuaternion<f32>| {
            let q_shortest = if q.w < 0.0 {
                -q.into_inner()
            } else {
                q.into_inner()
            };
            let q_lerped = Quaternion::identity().lerp(&q_shortest, alpha);

            // This normalization is IMPORTANT to avoid the quaternion from becoming non-unity
            UnitQuaternion::new_normalize(q_lerped)
        };

        // Do nlerp-based "leaking" to drive both quaternions towards identity
        self.leaky_gyro = nlerp_to_identity(self.leaky_gyro);
        self.leaky_pred = nlerp_to_identity(self.leaky_pred);

        axis_error.into()
    }
}

/// Faster approximate alternative to `quat.scaled_axis()`, only loses some magnitude at very large angles.
fn approximate_scaled_axis(quat: &UnitQuaternion<f32>) -> Vector3<f32> {
    let v = quat.vector();
    let n2 = v.norm_squared();
    v * (2.0 * (1.0 + n2 / 6.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Unit;

    #[test]
    fn approximate_scaled_axis_accuracy() {
        // Use an axis that is not aligned with any coordinate axis.
        let axis = Vector3::new(0.3, -0.5, 0.2);

        for angle_deg in [0.5f32, 1.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0] {
            let q =
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), angle_deg.to_radians());

            let exact = q.scaled_axis();
            let approx = approximate_scaled_axis(&q);

            // Ensure the vectors computed using either method are nearly identical
            assert_relative_eq!(approx, exact, epsilon = 2.0e-3);

            // The approximation must preserve the rotation axis exactly.
            assert_relative_eq!(approx.normalize(), exact.normalize(), epsilon = 1.0e-6);
        }
    }
}
