#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

use nalgebra::{ComplexField, SMatrix, Scalar, SimdValue};

struct VecMat<const N: usize, F: Scalar + SimdValue + ComplexField + Copy> {
    x: SMatrix<F, N, 1>,
    P: SMatrix<F, N, N>,
}

/// Extended `Nx`-dimensional kalman filter implementation utilizing the `nalgebra` library.
pub struct KalmanFilter<
    const Nx: usize,
    const Nu: usize,
    const Ny: usize,
    F: Scalar + SimdValue + ComplexField + Copy,
> {
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

impl<
        const Nx: usize,
        const Nu: usize,
        const Ny: usize,
        F: Scalar + SimdValue + ComplexField + Copy,
    > KalmanFilter<Nx, Nu, Ny, F>
{
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
            Ff,
            Fj,
            Hf,
            Hj,
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

    /// Predict new state using control input. If plant dynamics are time-dependent,
    /// this method (or `.predict`) must be called at the correct frequency.
    pub fn predict_with_input(&mut self, u: SMatrix<F, Nu, 1>) {
        match self.post.as_mut() {
            // Simple prediction, no new observations
            None => {
                self.prio.x = (self.Ff)(self.prio.x, u);
                let Fj = (self.Fj)(self.prio.x, u);
                self.prio.P = Fj * self.prio.P * Fj.transpose() + self.Q;
            }

            // Prediction based on new observations
            Some(post) => {
                // Finish calc for P_post and symmetrize
                post.P = post.P * self.prio.P;

                // Symmetrize
                post.P = (post.P + post.P.transpose()).scale(nalgebra::convert(0.5));

                // Update priors
                self.prio.x = (self.Ff)(post.x, u);
                let Fj = (self.Fj)(self.prio.x, u);
                self.prio.P = Fj * post.P * Fj.transpose() + self.Q;

                // Set posteriors to none
                self.post = None;
            }
        }
    }

    /// Update filter with new measurements.
    pub fn update(&mut self, y: &SMatrix<F, Ny, 1>) {
        self.update_with_input(y, SMatrix::zeros())
    }

    /// Update filter with new measurements and control input.
    pub fn update_with_input(&mut self, y: &SMatrix<F, Ny, 1>, u: SMatrix<F, Nu, 1>) {
        // Measurement prediction residual
        let y_res = y - (self.Hf)(self.prio.x, u);

        // Innovation (or pre-fit residual) covariance
        let Hj = (self.Hj)(self.prio.x, u);
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
