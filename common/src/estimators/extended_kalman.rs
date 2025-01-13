#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use nalgebra::{ComplexField, Quaternion, SMatrix, SVector, Scalar, SimdValue, Unit, UnitQuaternion, Vector3};

use crate::consts::GRAVITY;

struct VecMat<const N: usize, F: Scalar + SimdValue + ComplexField + Copy> {
    x: SMatrix<F, N, 1>,
    P: SMatrix<F, N, N>,
}

/// Extended `Nx`-dimensional kalman filter implementation utilizing the `nalgebra` library.
pub struct KalmanFilter<const Nx: usize, const Nu: usize, const Ny: usize, F: Scalar + SimdValue + ComplexField + Copy> {

    /// Model propagation function on the form `x[k+1] = f(x[k],u[k])`
    Ff: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Nx, 1>,

    /// Model propagation jacobian function. Should generate the
    /// jacobian of propagation function given current state and input.
    Fj: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Nx, Nx>,

    /// Measurement jacobian function. Should generate the
    /// jacobian of measurement function given current state and input.
    Hj: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Ny, Nx>,

    /// Measurement function `y[k] = h(x[k],u[k])`
    Hf: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Ny, 1>,

    /// Model noise covariance matrix.
    Q: SMatrix<F, Nx, Nx>,

    /// Measurement noise covariance matrix.
    R: SMatrix<F, Ny, Ny>,

    /// A priori state vector and covariance matrix.
    prio: VecMat<Nx, F>,

    /// A posteriori state vector and covariance matrix.
    post: Option<VecMat<Nx, F>>,

}

impl<const Nx: usize, const Nu: usize, const Ny: usize, F: Scalar + SimdValue + ComplexField + Copy> KalmanFilter<Nx, Nu, Ny, F> {
    /// Provide kalman filter with all initial values
    pub fn new(
        Ff: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Nx, 1>,
        Fj: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Nx, Nx>,
        Hf: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Ny, 1>,
        Hj: fn(SMatrix<F, Nx, 1>, SMatrix<F, Nu, 1>) -> SMatrix<F, Ny, Nx>,
        Q: SMatrix<F, Nx, Nx>,
        R: SMatrix<F, Ny, Ny>,
        x_init: SMatrix<F, Nx, 1>,
        P_init: SMatrix<F, Nx, Nx>,
    ) -> Self {
        Self {
            Ff,Fj,Hf,Hj,Q,R,
            prio: VecMat {
                x: x_init,
                P: P_init,
            },
            post: None,
        }
    }

    /// Predict new state. If plant dynamics are time-dependent,
    /// this method (or `.predict_with_input`) must be called at the correct frequency.
    pub fn predict(&mut self) {
        self.predict_with_input(SMatrix::zeros())
    }

    /// Predict new state using control input. If plant dynamics are time-dependent,
    /// this method (or `.predict`) must be called at the correct frequency.
    pub fn predict_with_input(&mut self, u: SMatrix<F, Nu, 1>) {
        match self.post.as_mut() {
            // Simple prediction, no new observations
            None => {
                self.prio.x = (self.Ff)(self.prio.x,u);
                let Fj = (self.Fj)(self.prio.x,u);
                self.prio.P = Fj * self.prio.P * Fj.transpose() + self.Q;
            }

            // Prediction based on new observations
            Some(post) => {
                // Finish calc for P_post and symmetrize
                post.P = post.P * self.prio.P;

                // Symmetrize
                post.P = (post.P + post.P.transpose()).scale(nalgebra::convert(0.5));

                // Update priors
                self.prio.x = (self.Ff)(post.x,u);
                let Fj = (self.Fj)(self.prio.x,u);
                self.prio.P = Fj * post.P * Fj.transpose() + self.Q;

                // Set posteriors to none
                self.post = None;
            }
        }
    }

    /// Update filter with new measurements.
    pub fn update(&mut self, y: &SMatrix<F, Ny, 1>) {
        self.update_with_input(y,SMatrix::zeros())
    }

    /// Update filter with new measurements and control input.
    pub fn update_with_input(&mut self, y: &SMatrix<F, Ny, 1>, u: SMatrix<F, Nu, 1>) {
        // Measurement prediction residual
        let y_res = y - (self.Hf)(self.prio.x,u);

        // Innovation (or pre-fit residual) covariance
        let Hj = (self.Hj)(self.prio.x,u);
        let S = Hj * self.prio.P * Hj.transpose() + self.R;

        // Optimal Kalman gain
        let Some(Sinv) = S.try_inverse() else { return };
        let K = self.prio.P * Hj.transpose() * Sinv;

        // Updated (a posteriori) estimate covariance
        self.post = Some(match self.post.as_mut() {
            Some(post) => VecMat {
                x: post.x + K * y_res,
                P: post.P - K * Hj,
            },
            None => VecMat {
                x: self.prio.x + K * y_res,
                P: SMatrix::identity() - K * Hj,
            },
        });
    }

    /// Get state vector `x`.
    /// Returns the posterior estimate if it exists, otherwise returns priori prediction.
    pub fn get_state(&self) -> SMatrix<F, Nx, 1> {
        self.post.as_ref().unwrap_or(&self.prio).x
    }
}

const N_STATES: usize = 16;
const N_INPUTS: usize = 6;
const N_OUTPUTS: usize = 3;

type X = SVector<f32, N_STATES>;
type U = SVector<f32, N_INPUTS>;
type Y = SVector<f32, N_OUTPUTS>;

struct StateVec {
    inner: SVector<f32, N_STATES>,
}

impl StateVec {
    fn new() -> Self {
        Self {
            inner: SVector::zeros(),
        }
    }

    fn position(&self) -> Vector3<f32> {
        Vector3::new(self.inner[0], self.inner[1], self.inner[2])
    }

    fn velocity(&self) -> Vector3<f32> {
        Vector3::new(self.inner[3], self.inner[4], self.inner[5])
    }

    fn orientation(&self) -> Unit<Quaternion<f32>> {
        UnitQuaternion::from_quaternion(Quaternion::from_parts(self.inner[6], Vector3::new(self.inner[7], self.inner[8], self.inner[9])))
    }

    fn gyro_bias(&self) -> Vector3<f32> {
        Vector3::new(self.inner[10], self.inner[11], self.inner[12])
    }

    fn accel_bias(&self) -> Vector3<f32> {
        Vector3::new(self.inner[13], self.inner[14], self.inner[15])
    }

    fn set_position(&mut self, pos: Vector3<f32>) {
        self.inner[0] = pos.x;
        self.inner[1] = pos.y;
        self.inner[2] = pos.z;
    }

    fn set_velocity(&mut self, vel: Vector3<f32>) {
        self.inner[3] = vel.x;
        self.inner[4] = vel.y;
        self.inner[5] = vel.z;
    }

    fn set_orientation(&mut self, quat: Unit<Quaternion<f32>>) {
        let [qx, qy, qz, qw] = (*quat.as_vector()).into();
        self.inner[6] = qw;
        self.inner[7] = qx;
        self.inner[8] = qy;
        self.inner[9] = qz;
    }

    fn set_gyro_bias(&mut self, bias: Vector3<f32>) {
        self.inner[10] = bias.x;
        self.inner[11] = bias.y;
        self.inner[12] = bias.z;
    }

    fn set_accel_bias(&mut self, bias: Vector3<f32>) {
        self.inner[13] = bias.x;
        self.inner[14] = bias.y;
        self.inner[15] = bias.z;
    }
}

/// State transition function for applying new IMU readings
fn f_imu(state: X, acc: Vector3<f32>, gyr: Vector3<f32>, gain: f32, dt: f32) -> X {

    // Input state vector into helper wrapper
    let mut state = StateVec { inner: state };

    // Correct IMU readings using biases
    let accel_body = acc - state.accel_bias();
    let gyro_body = gyr - state.gyro_bias();

    // Gravity vector (positive z-direction in NED frame)
    let gravity = Vector3::new(0.0, 0.0, GRAVITY);

    // Calculate acceleration in world frame
    let accel_world = state.orientation() * accel_body + gravity;

    // Update position and velocity predictions
    state.set_position(state.position() + state.velocity() * dt + 0.5 * accel_world * dt * dt);
    state.set_velocity(state.velocity() + accel_world * dt);

    // Compute rate of change for quaternion
    let omega_quat = *state.orientation() * Quaternion::from_parts(0.0, gyro_body) * 0.5 * dt;

    // Integrate to yield quaternion
    let mut int_quat = *state.orientation() + omega_quat;

    // Use gravity to correct attitude
    let expected_gravity = Vector3::new(0.0, 0.0, 1.0);

    // Compute the error between the measured and expected gravity vectors
    let accel_error = accel_world.normalize().cross(&expected_gravity);

    // Apply the correction to the orientation
    let accel_correction_quat = Quaternion::from_parts(0.0, accel_error * gain) * 0.5 * dt;
    int_quat = int_quat + accel_correction_quat;

    // Set the orientation in the state vector and return
    state.set_orientation(UnitQuaternion::from_quaternion(int_quat));
    state.inner
}

/// State transition function for applying new magnetometer readings
fn f_mag(state: X, mag: Vector3<f32>, gain: f32, dt: f32) -> X {

    // Input state vector into helper wrapper
    let mut state = StateVec { inner: state };

    // Normalize magnetometer measurement
    let mag = mag.normalize();

    // Project the magnetic field onto the horizontal plane (assuming z-axis is up)
    let mag_body_horizontal = Vector3::new(mag.x, mag.y, 0.0).normalize();

    // Reference magnetic field in the horizontal plane (assumed to be aligned with the x-axis)
    let mag_world_horizontal = Vector3::new(1.0, 0.0, 0.0);

    // Rotate the reference magnetic field to the body frame
    let mag_world_rotated = state.orientation().inverse() * mag_world_horizontal;

    // Compute the error between the measured and reference magnetic fields in the horizontal plane
    let error = mag_body_horizontal.cross(&mag_world_rotated);

    // Apply a correction to the orientation based on the error and gain
    let mag_correction_quat = Quaternion::from_parts(0.0, error * gain) * 0.5 * dt;
    let int_quat = *state.orientation() + mag_correction_quat;

    // Set the orientation in the state vector and return
    state.set_orientation(UnitQuaternion::from_quaternion(int_quat));
    state.inner
}

