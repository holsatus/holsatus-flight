use crate::{errors::CalibrationError, filters::rate_pid::FeedbackHandle, tasks::calibrator::{CalibratorState, Sensor}};

use super::{sens3d::{Calib3DType, SmallCalib3D}, MagCalib};

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MagCalState {
    Initialized,
    Collecting(MagCalCollecting),
    DoneSuccess(Calib3DType),
    DoneFailed(CalibrationError),
}

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagCalCollecting {
    pub sample: [f32; 3],
    pub mean_distance: f32,
}

pub async fn calibrate_mag<'a>(config: MagCalib, sensor_id: u8, feedback: FeedbackHandle<'a, MagCalState>) -> Result<Calib3DType, CalibrationError> {

    feedback.send(MagCalState::Initialized);

    let mut calibrator = MagCalibrator::<26>::new()
        .pre_scaler(config.pre_scalar)
        .num_neighbors(1);

    // Input channels
    let mut rcv_raw_mag = crate::signals::RAW_MULTI_MAG_DATA.get(sensor_id as usize)
        .ok_or(CalibrationError::MagInvalidId)?
        .receiver();

    let mut failed_calibrations = 0;
    let mut dropped = 0;

    'calibration: loop {
        Timer::after_millis(50).await;

        let Some(data) = rcv_raw_mag.try_changed() else {
            if dropped > config.max_dropped {
                error!("Dropped too many samples, aborting calibration");
                feedback.send(MagCalState::DoneFailed(CalibrationError::MagMaxDropped));
                return Err(CalibrationError::MagMaxDropped);
            } else {
                dropped += 1;
                Timer::after_millis(100).await;
                continue 'calibration;
            }
        };

        calibrator.evaluate_sample(data);
        feedback.send(MagCalState::Collecting(MagCalCollecting {
            sample: data,
            mean_distance: calibrator.get_mean_distance(),
        }));

        info!("Received new sample: {:?}", data);
        info!("Mean distance: {}", calibrator.get_mean_distance());

        if calibrator.get_mean_distance() > 0.018 {

            // If any outliers could be removed, continue
            // let mut removed_outliers = false;
            // while calibrator.remove_outliers() {
            //     info!("Removed outliers, recalculating mean distance");
            //     removed_outliers = true;
            // }
            // if removed_outliers {
            //     continue 'calibration;
            // }

            if let Some((bias, scale)) = calibrator.perform_calibration() {

                info!("Calibration complete, bias: {:?}, scale: {:?}", bias, scale);
                feedback.send(MagCalState::DoneSuccess(Calib3DType::Small(SmallCalib3D {
                    bias: Some(bias.into()),
                    scale: Some(scale.into()),
                })));
                return Ok(Calib3DType::Small(SmallCalib3D {
                    bias: Some(bias.into()),
                    scale: Some(scale.into()),
                }));
            } else {
                failed_calibrations += 1;
                if failed_calibrations > 5 {
                    feedback.send(MagCalState::DoneFailed(CalibrationError::MagBadFit));
                    error!("Failed to calibrate magnetometer, aborting");
                    return Err(CalibrationError::MagBadFit);
                }
            }
        }
    }
}


use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, signal::Signal, watch::Sender};
use embassy_time::Timer;
use nalgebra::{ComplexField, SMatrix, SMatrixView, SVector, Vector3};

/// Lightweight least squares approach to
/// determining the offset and scaling
/// factors for magnetometer calibration.
/// Also includes the capability to automatically
/// collect good data points, using a `const`-sized
/// buffer matrix, and a k-nearest neighbors.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 6>,
    matrix_filled: usize,
    mean_distance: f32,
    pre_scaler: f32,
    k: usize,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::from_element(1.0),
            matrix_filled: Default::default(),
            mean_distance: Default::default(),
            pre_scaler: 1.,
            k: 2, // Works well in testing
        }
    }
}

impl<const N: usize> MagCalibrator<N> {
    /// Create a new calibrator instance.
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the number of `k` neighbors to calculate distance to.
    pub fn num_neighbors(self, k: usize) -> Self {
        Self { k, ..self }
    }

    /// Configure sample pre scaler, prevents ill-conditioning if given
    /// a value close to the expected magnitude of the magnetic field strength.
    pub fn pre_scaler(self, pre_scaler: f32) -> Self {
        Self { pre_scaler, ..self }
    }

    /// Calculates mean distance to the `k` nearest neighbors.
    /// A smaller number means the point is "similar" to its neighbors.
    fn mean_distance_from_single(&self, vec: SMatrix<f32, 1, 3>) -> f32 {
        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);

        // Distance to every other point
        let mut squared_dists: [f32; N] = [0.; N];
        matrix_view.row_iter().enumerate().for_each(|(j, cmp)| {
            let diff = vec - cmp;
            squared_dists[j] = diff.dot(&diff).sqrt(); // ?
        });

        // Sort floats and return mean distance to nearest neighbors
        squared_dists.sort_unstable_by(|a, b| a.total_cmp(b));
        squared_dists
            .iter()
            .take(self.k + 1)
            .rfold(0., |a, &b| a + b)
            / N as f32
    }

    /// Calculates mean squared distance to the `k` nearest neighbors
    /// between all `N` row vectors in the internal buffer.
    /// A smaller number means a point is "similar" to its neighbors.
    fn mean_distance_from_all(&self) -> [f32; N] {
        let mut mean_dist: [f32; N] = [0.; N];

        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);
        matrix_view.row_iter().enumerate().for_each(|(i, row)| {
            mean_dist[i] = self.mean_distance_from_single(row.into());
        });
        mean_dist
    }

    /// Returns index of vector with the lowest squared distance
    /// Is used when replacing the least useful value in the array.
    fn lowest_mean_distance_by_index(&mut self) -> (usize, f32) {
        let mean_dist = self.mean_distance_from_all();

        // Set mean distance now that we are at it
        self.mean_distance = mean_dist.iter().rfold(0., |a, &b| a + b) / N as f32;

        // Obtain index for lowest mean distance
        mean_dist
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(index, value)| (index, *value))
            .unwrap()
    }

    /// Evaluates whether the new sample should replace one already in the buffer.
    pub fn evaluate_sample(&mut self, x: [f32; 3]) {
        self.evaluate_sample_vec(Vector3::from(x))
    }

    /// Add a sample if it is deemed more useful than the least useful sample.
    pub fn evaluate_sample_vec(&mut self, x: Vector3<f32>) {
        // Ensure all entries are normal (not Inf or NaN)
        if !x.iter().all(|e| e.is_normal()) {
            return;
        }
        // Check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            self.add_sample_at(self.matrix_filled, x);
            self.matrix_filled += 1;
        }
        // Otherwise check which sample may be best to replace
        else {
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let sample_mean_dist = self.mean_distance_from_single(x.transpose());
            if low_mean_dist < sample_mean_dist {
                self.add_sample_at(low_index, x);
            }
        }
    }

    /// Insert a sample vector into `index` row of buffer matrix.
    fn add_sample_at(&mut self, index: usize, sample: Vector3<f32>) {
        if index < N {
            self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
            self.matrix[(index, 1)] = sample[1] / self.pre_scaler;
            self.matrix[(index, 2)] = sample[2] / self.pre_scaler;
        }
    }

    /// Get mean distance value between samples in matrix buffer.
    pub fn get_mean_distance(&self) -> f32 {
        if self.matrix_filled < N {
            0.0
        } else {
            self.mean_distance
        }
    }

    // TODO - This is currently broken, and may not be the best way to remove outliers
    pub fn remove_outliers(&mut self) -> bool {

        let mut norms = SVector::<f32, N>::zeros();
        (self.matrix.row_iter()).zip(norms.iter_mut()).take(self.matrix_filled).for_each(|(row, norms)| {
            *norms = Vector3::new(row[0], row[1], row[2]).norm();
        });

        let variance = norms.variance();

        for (i, norm) in norms.iter().enumerate() {
            if (norm - 1.0).abs() > 2. * variance.sqrt() {
                for j in i..N-1 {
                    let next = self.matrix.row(j+1).into_owned();
                    self.matrix.row_mut(j).copy_from(&next);
                }

                self.matrix_filled -= 1;
            }
        }

        // Return true if any were removed
        return self.matrix_filled < N;
    }

    /// Try to calculate calibration offset and scale values. Returns None if
    /// it was not possible to calculate the pseudo inverse, or if some of the
    /// parameters are `NaN`. In that case it would be best to restart the whole
    /// calibration and collect new samples. The tuple contains (offset , scale).
    pub fn perform_calibration(&mut self) -> Option<([f32; 3], [f32; 3])> {
        // Calculate column 4 and 5 of H matrix
        self.matrix.row_iter_mut().for_each(|mut mag| {
            mag[3] = -mag[1] * mag[1];
            mag[4] = -mag[2] * mag[2];
        });

        // Calculate W vector
        let mut w: SMatrix<f32, N, 1> = SMatrix::from_element(0.0);
        self.matrix
            .row_iter()
            .enumerate()
            .for_each(|(i, row)| w[i] = row[0] * row[0]);

        // Perform least squares using pseudo inverse
        let x =
            (self.matrix.transpose() * self.matrix).try_inverse()? * self.matrix.transpose() * w;

        // Calculate offsets and scale factors
        let off = [x[0] / 2., x[1] / (2. * x[3]), x[2] / (2. * x[4])];
        let temp = x[5] + (off[0] * off[0]) + x[3] * (off[1] * off[1]) + x[4] * (off[2] * off[2]);
        let scale = [temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt()];

        // Check that off and scale vectors contain valid values
        for x in off.iter().chain(scale.iter()) {
            if !x.is_finite() {
                return None;
            }
        }

        // Unscale the offset values
        let off= off.map(|x| x * self.pre_scaler);

        // TODO Add option for low-pass filtering this result
        Some((off, scale))
    }
}
