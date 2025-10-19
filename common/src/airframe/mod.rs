use nalgebra::{Point3, SMatrix, Vector3, Vector4};

pub const DEV_QUAD_MOTOR_SETUP: MotorSetup<4> = MotorSetup::quad_x_basic(0.165, 0.225, 0.1, false);

struct Motor {
    /// The position of this motor relative to the vehicle center of mass.
    pub position: Point3<f32>,

    /// The direction along which this motor will produce its thrust.
    pub direction: Vector3<f32>,

    /// The reaction torque caused by the propeller spinning. The sign
    /// of this value indicates its direction of spin. Positive is CW.
    pub reaction: f32,
}

pub struct MotorSetup<const N: usize> {
    motor: [Motor; N],
}

impl MotorSetup<4> {
    pub const fn quad_x_basic(
        x_dist: f32,
        y_dist: f32,
        reaction: f32,
        outspin: bool,
    ) -> MotorSetup<4> {
        let x_half_dist = x_dist / 2.0;
        let y_half_dist = y_dist / 2.0;
        let dir = if outspin { -1.0 } else { 1.0 };
        MotorSetup {
            motor: [
                Motor {
                    // back right
                    position: Point3::new(-x_half_dist, y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: reaction * dir, // + (CW)
                },
                Motor {
                    // front right
                    position: Point3::new(x_half_dist, y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: -reaction * dir, // - (CW)
                },
                Motor {
                    // back left
                    position: Point3::new(-x_half_dist, -y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: -reaction * dir, // - (CW)
                },
                Motor {
                    // front left
                    position: Point3::new(x_half_dist, -y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: reaction * dir, // + (CW)
                },
            ],
        }
    }

    pub const fn quad_p_basic(
        x_motor_spacing: f32,
        y_motor_spacing: f32,
        reaction: f32,
        reverse: bool,
    ) -> MotorSetup<4> {
        let x_half_dist = x_motor_spacing / 2.0;
        let y_half_dist = y_motor_spacing / 2.0;
        let dir = if reverse { -1.0 } else { 1.0 };
        MotorSetup {
            motor: [
                Motor {
                    // right
                    position: Point3::new(0.0, y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: reaction * dir,
                },
                Motor {
                    // front
                    position: Point3::new(x_half_dist, 0.0, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: -reaction * dir,
                },
                Motor {
                    // back
                    position: Point3::new(-x_half_dist, 0.0, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: -reaction * dir,
                },
                Motor {
                    // left
                    position: Point3::new(0.0, -y_half_dist, 0.0),
                    direction: Vector3::new(0.0, 0.0, -1.0),
                    reaction: reaction * dir,
                },
            ],
        }
    }
}

impl<const N: usize> MotorSetup<N> {
    pub fn into_mixing_matrix(&self) -> Option<SMatrix<f32, N, 4>> {
        assert!(
            N >= 4,
            "Motor mixing matrix calculation not supported for fewer than 4 rotors"
        );

        // Build the effectiveness matrix
        let mut matrix = SMatrix::<f32, 4, N>::zeros();

        for (i, motor) in self.motor.iter().enumerate() {
            // Ensure the direction is a unit vector
            let direction_normalized = motor.direction.normalize();

            // Torque from the lever arm (r x d)
            let lever_arm_torque = motor.position.coords.cross(&direction_normalized);

            // Reaction torque from propeller spin (d * c)
            let reaction_torque = direction_normalized * motor.reaction;

            // Total torque is the sum of both
            let total_torque_vector = lever_arm_torque + reaction_torque;

            // Create the column vector for this motor using the total torque
            let motor_contribution_column = Vector4::new(
                total_torque_vector.x,
                total_torque_vector.y,
                total_torque_vector.z,
                direction_normalized.z,
            );

            matrix.set_column(i, &motor_contribution_column);
        }

        // Calculate the right pseudo inverse.
        let aat = &matrix * matrix.transpose();
        let aat_inv = aat.try_inverse()?;
        let mixing_matrix = matrix.transpose() * aat_inv;

        // The mixing matrix is the inverse of effectiveness
        Some(mixing_matrix)
    }
}
