use embassy_time::{with_timeout, Duration, Ticker};
use nalgebra::{ComplexField, SMatrix, Vector3};

use crate::{config::Calibration, constants::G_GRAVITY, messaging as msg, sensors::imu::types::ImuData6Dof};

/// Routine to calibrate gyroscopes, by calculating their bias.
#[embassy_executor::task]
pub async fn acc_calibration() -> ! {

    // Input channels
    let mut rcv_imu_sensor_array: [_; crate::N_IMU] = core::array::from_fn(|i| msg::IMU_SENSOR[i].receiver().unwrap());
    let mut rcv_start_acc_calib = msg::CMD_START_ACC_CALIB.receiver().unwrap();

    // Output channels
    let snd_acc_calibing = msg::ACC_CALIBRATING.sender();
    let snd_imu_config = msg::CFG_IMU_CONFIG.sender();
    
    'infinite: loop {
        
        // Wait for the start signal
        snd_acc_calibing.send(false);
        if !rcv_start_acc_calib.changed().await { continue 'infinite };
        snd_acc_calibing.send(true);

        defmt::info!("[ACC CALIB]: Starting calibration on {} accelerometers", crate::N_IMU);

        // Array of calibrator instances
        let mut sensor_calibrator = [AccCalibrator::<100>::new(); crate::N_IMU];
        
        let mut timeout_count = 0;
        let mut done_count = 0;

        let mut ticker = Ticker::every(Duration::from_hz(50));
        
        // Calibration loop
        'calibration: loop {

            ticker.next().await;

            // Iterate through all IMUs
            'iter_sensors: for (id, (rcv_imu, calibrator)) in rcv_imu_sensor_array.iter_mut().zip(sensor_calibrator.iter_mut()).enumerate() {

                // If the calibration has timed out or is complete, skip this sensor
                if calibrator.has_timeout() || calibrator.buffer_is_full() {
                    continue 'iter_sensors;
                }

                // Await IMU data, or detect time out
                match with_timeout(Duration::from_millis(50), rcv_imu.changed()).await {

                    // Insert IMU data into the calibrator
                    Ok(imu_data) => {
                        calibrator.collect(imu_data);
                        if calibrator.is_done() {
                            defmt::warn!("[ACC CALIB]: Acc calibration complete for sensor {}", id);
                            defmt::warn!("[ACC CALIB]: Got the values: {:?}", defmt::Debug2Format(&calibrator.measured_gravity) );
                            done_count += 1;
                        }
                    },

                    // IMU Timed out
                    Err(_) => {
                        calibrator.set_timeout();
                        defmt::warn!("[ACC CALIB]: Acc calibration timed out for sensor {}", id);
                        timeout_count += 1; 
                    },
                }

                // Check if calibration has been stopped by user
                if rcv_start_acc_calib.try_changed() == Some(false) {
                    defmt::info!("[ACC CALIB]: Calibration process stopped by user");
                    continue 'infinite;
                }
            }

            // If all sensors have timed out or are done, break calibration loop
            if timeout_count + done_count == crate::N_IMU {
                break 'calibration;
            }
        }

        // Get existing calibration data or create new
        let mut imu_config = msg::CFG_IMU_CONFIG.spin_get().await;

        // Publish result of each sensor if calibration was successful
        for (id, (opt_config, calibrator)) in imu_config.iter_mut().zip(sensor_calibrator.iter()).enumerate() {
            match (calibrator.calib, opt_config) {
                (Some(calib), Some(config)) => {
                    defmt::info!("[ACC CALIB]: Sensor {} calibration is valid.", id);
                    config.acc_cal = Some(calib);
                },
                (None, Some(_)) => defmt::warn!("[ACC CALIB]: Sensor {} calibration is invalid", id),
                (Some(_), None) => defmt::warn!("[ACC CALIB]: Sensor {} calibration is valid, but no configuration for the sensor exists", id),
                (None, None) => {/* This is expected for non-existant sensors */},
            }
        }

        snd_imu_config.send(imu_config);

        // Ensure any recent request to calibrate is marked as seen
        let _ = rcv_start_acc_calib.try_get();
    }
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
    has_timed_out: bool,
    count: usize, 
    calib: Option<Calibration>,
}

impl <const N: usize> AccCalibrator<N> {
    pub fn new() -> Self {
        Self {
            measured_gravity: MeasuredDirections::default(),
            directions_measured: Direction::empty(),
            acc_buffer: SMatrix::zeros(),
            has_timed_out: false,
            count: 0,
            calib: None,
        }
    }

    pub fn collect(&mut self, meas: ImuData6Dof) {
        let acc = meas.acc;

        // Determine the direction of gravity
        let direction = if acc.x > acc.y.abs()*10. && acc.x > acc.z.abs()*10. {
                Direction::X_PLUS
            } else if acc.x < -acc.y.abs()*10. && acc.x < -acc.z.abs()*10. {
                Direction::X_MINUS
            } else if acc.y > acc.x.abs()*10. && acc.y > acc.z.abs()*10. {
                Direction::Y_PLUS
            } else if acc.y < -acc.x.abs()*10. && acc.y < -acc.z.abs()*10. {
                Direction::Y_MINUS
            } else if acc.z > acc.x.abs()*10. && acc.z > acc.y.abs()*10. {
                Direction::Z_PLUS
            } else if acc.z < -acc.x.abs()*10. && acc.z < -acc.y.abs()*10. {
                Direction::Z_MINUS
            } else {
                self.count = 0;
                return;
            };

        // defmt::println!("Direction: {:?}  from {:?}", defmt::Debug2Format(&direction), defmt::Debug2Format(&acc));

        // If the direction has already been measured, skip
        if self.directions_measured.contains(direction) {
            return;
        }

        self.acc_buffer.set_column(self.count, &acc);
        self.count += 1;

        // If variance in buffer is too high, reset
        if let Some(var) = self.acc_variance() && var > 0.01 {
            defmt::warn!("[ACC CALIB]: Resetting {:?} measurement due to high variance", defmt::Debug2Format(&direction));
            self.count = 0;
            return;
        }

        // If buffer is not full, skip
        if !self.buffer_is_full() {
            return;
        }

        let mean = self.acc_mean();
        self.directions_measured.insert(direction);

        match direction {
            Direction::X_PLUS =>  self.measured_gravity.x_plus = mean.x,
            Direction::X_MINUS => self.measured_gravity.x_minus = mean.x,
            Direction::Y_PLUS =>  self.measured_gravity.y_plus = mean.y,
            Direction::Y_MINUS => self.measured_gravity.y_minus = mean.y,
            Direction::Z_PLUS =>  self.measured_gravity.z_plus = mean.z,
            Direction::Z_MINUS => self.measured_gravity.z_minus = mean.z,
            _ => { defmt::unreachable!("This directions flag should only contains a single instance") },
        }

        self.count = 0;
        defmt::info!("[ACC CALIB]: Finished measuring {:?}, now have {:?}", defmt::Debug2Format(&direction), defmt::Debug2Format(&self.directions_measured));

        if self.is_done() {

            defmt::info!("[ACC CALIB]: Calibration finished!");

            let calib = &self.measured_gravity;
    
            let offset = Vector3::new(
                (calib.x_plus + calib.x_minus) / 2.0,
                (calib.y_plus + calib.y_minus) / 2.0,
                (calib.z_plus + calib.z_minus) / 2.0,
            );
        
            let scale = Vector3::new(
                G_GRAVITY / (calib.x_plus - calib.x_minus) * 2.0,
                G_GRAVITY / (calib.y_plus - calib.y_minus) * 2.0,
                G_GRAVITY / (calib.z_plus - calib.z_minus) * 2.0,
            );
        
            self.calib = Some(Calibration { offset, scale });
        }
    }

    pub fn has_timeout(&self) -> bool {
        self.has_timed_out
    }

    pub fn set_timeout(&mut self) {
        self.has_timed_out = true;
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
