#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use nalgebra::{ComplexField, SMatrix, Scalar, SimdValue};

struct VecMat<const N: usize, F: Scalar + SimdValue + ComplexField + Copy> {
    x: SMatrix<F, N, 1>,
    P: SMatrix<F, N, N>,
}

/// Linear state-space `Nx`-dimensional Kalman filter implementation utilizing the `nalgebra` library.
pub struct KalmanFilter<
    const Nx: usize,
    const Nu: usize,
    const Ny: usize,
    F: Scalar + SimdValue + ComplexField + Copy,
> {
    // Model propagation matrix
    A: SMatrix<F, Nx, Nx>,

    // Input matrix
    B: SMatrix<F, Nx, Nu>,

    // Output matrix
    C: SMatrix<F, Ny, Nx>,

    // Model noise covariance matrix
    Q: SMatrix<F, Nx, Nx>,

    // Model noise covariance matrix
    R: SMatrix<F, Ny, Ny>,

    // A priori state vector and covariance matrix
    prio: VecMat<Nx, F>,

    // A posteriori state vector and covariance matrix
    post: Option<VecMat<Nx, F>>,
}

impl<
        const Nx: usize,
        const Nu: usize,
        const Ny: usize,
        F: Scalar + SimdValue + ComplexField + Copy,
    > KalmanFilter<Nx, Nu, Ny, F>
{
    /// Provide kalman filter with all initial values
    pub fn new(
        A: SMatrix<F, Nx, Nx>,
        B: SMatrix<F, Nx, Nu>,
        C: SMatrix<F, Ny, Nx>,
        Q: SMatrix<F, Nx, Nx>,
        R: SMatrix<F, Ny, Ny>,
        x_init: SMatrix<F, Nx, 1>,
        P_init: SMatrix<F, Nx, Nx>,
    ) -> Self {
        Self {
            A,
            B,
            C,
            Q,
            R,
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

    /// Predict new state using input. If plant dynamics are time-dependent,
    /// this method (or `.predict`) must be called at the correct frequency.
    pub fn predict_with_input(&mut self, u: SMatrix<F, Nu, 1>) {
        match self.post.as_mut() {
            // Simple prediction, no new observations
            None => {
                self.prio.x = self.A * self.prio.x + self.B * u;
                self.prio.P = self.A * self.prio.P * self.A.transpose() + self.Q;
            }

            // Prediction based on new observations
            Some(post) => {
                // Update priors
                self.prio.x = self.A * post.x + self.B * u;
                self.prio.P = self.A * post.P * self.A.transpose() + self.Q;

                // Set posteriors to none
                self.post = None;
            }
        }
    }

    /// Update filter with new measurements
    pub fn update(&mut self, y: &SMatrix<F, Ny, 1>) {
        // Measurement prediction residual
        let y_res = y - self.C * self.prio.x;

        // Innovation (or pre-fit residual) covariance
        let S = self.C * self.prio.P * self.C.transpose() + self.R;

        // Optimal Kalman gain
        let Some(Sinv) = S.try_inverse() else { return };
        let K = self.prio.P * self.C.transpose() * Sinv;

        // Updated (a posteriori) estimate covariance
        self.post = Some(match self.post.as_mut() {
            Some(post) => VecMat {
                x: post.x + K * y_res,
                P: post.P - K * self.C,
            },
            None => VecMat {
                x: self.prio.x + K * y_res,
                P: SMatrix::identity() - K * self.C,
            },
        });
    }

    /// Get state vector `x`.
    /// Returns the posterior estimate if it exists, otherwise returns priori prediction.
    pub fn get_state(&self) -> SMatrix<F, Nx, 1> {
        self.post.as_ref().unwrap_or(&self.prio).x
    }
}
