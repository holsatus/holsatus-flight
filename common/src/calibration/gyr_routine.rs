use super::GyrCalib;
use crate::{errors::CalibrationError, signals as s, types::measurements::Imu6DofData};

use embassy_time::{Duration, Ticker};
use nalgebra::{SMatrix, Vector3};

/// Routine to calibrate gyroscopes, by calculating their bias.
pub async fn calibrate_gyr_bias(
    config: GyrCalib,
    sensor_id: u8,
) -> Result<Vector3<f32>, CalibrationError> {
    const ID: &str = "gyr_bias_calib";

    const ACC_BUFFER_SIZE: usize = 50;
    const ACC_SAMPLE_DIV: usize = 4;

    // Input channels
    let mut rcv_raw_imu = s::RAW_MULTI_IMU_DATA
        .get(sensor_id as usize)
        .ok_or(CalibrationError::GyrInvalidId)?
        .receiver();

    info!(
        "{}: Starting bias calibration on sensor {}, max variance of {} ",
        ID, sensor_id, config.max_var
    );

    // Array of calibrator instances
    let mut calibrator = GyrCalibrator::<ACC_BUFFER_SIZE>::new(ACC_SAMPLE_DIV);

    // Determine how frequently we need to sample
    let frequency = (ACC_BUFFER_SIZE * ACC_SAMPLE_DIV) as u64 / config.duration_s as u64;
    let mut ticker = Ticker::every(Duration::from_hz(frequency));

    // Number of IMU measurements that were "dropped"
    let mut num_dropped = 0;

    // Mark any stale data as seen
    _ = rcv_raw_imu.try_get();

    // Calibration loop
    let acc_variance = 'calibration: loop {
        ticker.next().await;

        let Some(imu_data) = rcv_raw_imu.try_changed() else {
            if num_dropped > config.max_dropped {
                Err(CalibrationError::GyrMaxDropped)?
            } else {
                num_dropped += 1;
                continue 'calibration;
            }
        };

        if let Some(acc_variance) = calibrator.collect(imu_data) {
            break 'calibration acc_variance;
        }
    };

    // Get existing calibration data or create new
    info!(
        "{}: Gyr calibration complete for sensor {} with variance {}",
        ID, sensor_id, acc_variance
    );

    // Ensure variance is within limits
    if acc_variance > config.max_var {
        Err(CalibrationError::GyrHighVariance)?
    }

    // All good, return bias
    Ok(calibrator.get_bias())
}

#[derive(Debug, Clone, Copy)]
struct GyrCalibrator<const N: usize> {
    gyr_bias_sum: Vector3<f32>,
    acc_buffer: SMatrix<f32, 3, N>,
    acc_sample_div: usize,
    sample_count: usize,
}

// TODO, each calibrator could calculate the variance over a smaller number of sampels,
// and average the result. This enables us to use a much smaller buffer, while still
// getting a good estimate of the variance.

impl<const N: usize> GyrCalibrator<N> {
    pub fn new(acc_sample_div: usize) -> Self {
        Self {
            gyr_bias_sum: Vector3::zeros(),
            acc_buffer: SMatrix::zeros(),
            acc_sample_div,
            sample_count: 0,
        }
    }

    pub fn collect(&mut self, meas: Imu6DofData<f32>) -> Option<f32> {
        self.gyr_bias_sum += Vector3::from(meas.gyr);

        if self.sample_count % self.acc_sample_div == 0 {
            self.acc_buffer
                .set_column(self.sample_count / self.acc_sample_div, &meas.acc.into());
        }

        self.sample_count += 1;
        self.acc_variance()
    }

    pub fn get_bias(&self) -> Vector3<f32> {
        self.gyr_bias_sum / self.sample_count as f32
    }

    pub fn acc_variance(&self) -> Option<f32> {
        if self.sample_count / self.acc_sample_div >= N {
            Some(self.acc_buffer.column_variance().norm())
        } else {
            None
        }
    }
}
