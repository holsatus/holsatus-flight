use super::{sens3d::Calib3DType, AccCalib};

use crate::*;

use embassy_time::{Duration, Ticker};
use nalgebra::{SMatrix, Vector3};

use crate::{
    calibration::sens3d::SmallCalib3D, consts::GRAVITY, errors::CalibrationError, signals as s,
    types::measurements::Imu6DofData,
};

/// Routine to calibrate gyroscopes, by calculating their bias.
pub async fn calibrate_acc(
    config: AccCalib,
    sensor_id: u8,
) -> Result<Calib3DType, CalibrationError> {
    const ID: &str = "acc_calib";

    const ACC_BUFFER_SIZE: usize = 50;

    // Input channels
    let mut rcv_raw_imu = s::RAW_MULTI_IMU_DATA
        .get(sensor_id as usize)
        .ok_or(CalibrationError::AccInvalidId)?
        .receiver();

    info!(
        "{}: Starting accelerometer calibration on sensor {}, max std deviation of {} ",
        ID, sensor_id, config.max_var
    );

    // Array of calibrator instances
    let mut calibrator = AccCalibrator::<ACC_BUFFER_SIZE>::new(config.max_var);

    let mut num_dropped = 0;

    let mut ticker = Ticker::every(Duration::from_hz(25));

    // Mark any stale data as seen
    _ = rcv_raw_imu.try_get();

    // Calibration loop
    let calib = 'calibration: loop {
        ticker.next().await;

        let Some(imu_data) = rcv_raw_imu.try_changed() else {
            if num_dropped > config.max_dropped {
                Err(CalibrationError::AccMaxDropped)?
            } else {
                num_dropped += 1;
                continue 'calibration;
            }
        };

        match calibrator.collect(imu_data) {
            None => continue 'calibration,
            Some(calib) => break 'calibration calib,
        }
    };

    // Get existing calibration data or create new
    info!("{}: Gyr calibration complete for sensor {}", ID, sensor_id);

    // All good, return bias
    Ok(calib)
}

#[derive(Debug, Clone, Copy, Default)]
struct MeasuredDirections {
    pub x_plus: f32,
    pub x_minus: f32,
    pub y_plus: f32,
    pub y_minus: f32,
    pub z_plus: f32,
    pub z_minus: f32,
}

bitflags::bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq)]
    struct Direction: u8 {
        const X_PLUS = 1 << 0;
        const X_MINUS = 1 << 1;
        const Y_PLUS = 1 << 2;
        const Y_MINUS = 1 << 3;
        const Z_PLUS = 1 << 4;
        const Z_MINUS = 1 << 5;
        const ALL = 0b111111;
    }
}

#[derive(Debug, Clone, Copy)]
struct AccCalibrator<const N: usize> {
    measured_gravity: MeasuredDirections,
    directions_measured: Direction,
    acc_buffer: SMatrix<f32, 3, N>,
    max_variance: f32,
    count: usize,
}

impl<const N: usize> AccCalibrator<N> {
    pub fn new(max_variance: f32) -> Self {
        Self {
            measured_gravity: MeasuredDirections::default(),
            directions_measured: Direction::empty(),
            acc_buffer: SMatrix::zeros(),
            max_variance,
            count: 0,
        }
    }

    pub fn collect(&mut self, meas: Imu6DofData<f32>) -> Option<Calib3DType> {
        let acc = meas.acc;

        // Determine the direction of gravity
        let direction = if acc[0] > acc[1].abs() * 10. && acc[0] > acc[2].abs() * 10. {
            Direction::X_PLUS
        } else if acc[0] < -acc[1].abs() * 10. && acc[0] < -acc[2].abs() * 10. {
            Direction::X_MINUS
        } else if acc[1] > acc[0].abs() * 10. && acc[1] > acc[2].abs() * 10. {
            Direction::Y_PLUS
        } else if acc[1] < -acc[0].abs() * 10. && acc[1] < -acc[2].abs() * 10. {
            Direction::Y_MINUS
        } else if acc[2] > acc[0].abs() * 10. && acc[2] > acc[1].abs() * 10. {
            Direction::Z_PLUS
        } else if acc[2] < -acc[0].abs() * 10. && acc[2] < -acc[1].abs() * 10. {
            Direction::Z_MINUS
        } else {
            self.count = 0;
            return None;
        };

        // defmt::println!("Direction: {:?}  from {:?}", defmt::Debug2Format(&direction), defmt::Debug2Format(&acc));

        // If the direction has already been measured, skip
        if self.directions_measured.contains(direction) {
            return None;
        }

        self.acc_buffer.set_column(self.count, &acc.into());
        self.count += 1;

        // If variance in buffer is too high, reset

        match self.acc_variance() {
            // If variance is within limits, continue
            Some(var) if var < self.max_variance => {}
            // If variance is too high, reset
            Some(_) => {
                #[cfg(feature = "defmt")]
                let to_print = defmt::Debug2Format(&direction);
                #[cfg(not(feature = "defmt"))]
                let to_print = direction;

                warn!(
                    "[ACC CALIB]: Resetting {:?} measurement due to high variance",
                    to_print
                );
                self.count = 0;
                return None;
            }
            // If buffer is not full, return
            None => return None,
        }

        let mean = self.acc_mean();
        self.directions_measured.insert(direction);

        match direction {
            Direction::X_PLUS => self.measured_gravity.x_plus = mean.x,
            Direction::X_MINUS => self.measured_gravity.x_minus = mean.x,
            Direction::Y_PLUS => self.measured_gravity.y_plus = mean.y,
            Direction::Y_MINUS => self.measured_gravity.y_minus = mean.y,
            Direction::Z_PLUS => self.measured_gravity.z_plus = mean.z,
            Direction::Z_MINUS => self.measured_gravity.z_minus = mean.z,
            _ => {
                unreachable!("This directions flag should only contains a single instance")
            }
        }

        self.count = 0;

        #[cfg(feature = "defmt")]
        info!(
            "[ACC CALIB]: Finished measuring {:?}, now have {:?}",
            defmt::Debug2Format(&direction),
            defmt::Debug2Format(&self.directions_measured)
        );
        #[cfg(not(feature = "defmt"))]
        info!(
            "[ACC CALIB]: Finished measuring {:?}, now have {:?}",
            direction, self.directions_measured
        );

        if !self.is_done() {
            return None;
        }

        info!("[ACC CALIB]: Calibration finished!");

        let calib = &self.measured_gravity;

        let offset = Vector3::new(
            (calib.x_plus + calib.x_minus) / 2.0,
            (calib.y_plus + calib.y_minus) / 2.0,
            (calib.z_plus + calib.z_minus) / 2.0,
        );

        let scale = Vector3::new(
            GRAVITY / (calib.x_plus - calib.x_minus) * 2.0,
            GRAVITY / (calib.y_plus - calib.y_minus) * 2.0,
            GRAVITY / (calib.z_plus - calib.z_minus) * 2.0,
        );

        Some(Calib3DType::Small(SmallCalib3D {
            bias: Some(offset),
            scale: Some(scale),
        }))
    }

    pub fn acc_variance(&self) -> Option<f32> {
        if self.buffer_is_full() {
            Some(self.acc_buffer.column_variance().norm())
        } else {
            None
        }
    }

    pub fn acc_mean(&self) -> Vector3<f32> {
        self.acc_buffer.column_mean()
    }

    pub fn buffer_is_full(&self) -> bool {
        self.count >= N
    }

    pub fn is_done(&self) -> bool {
        self.directions_measured.contains(Direction::ALL)
    }
}
