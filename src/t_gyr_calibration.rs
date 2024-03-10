use embassy_time::{with_timeout, Duration, Ticker};
use nalgebra::{SMatrix, Vector3};

use crate::{config::Calibration, messaging as msg, sensors::imu::types::ImuData6Dof};

/// Routine to calibrate gyroscopes, by calculating their bias.
#[embassy_executor::task]
pub async fn gyr_calibration() -> ! {

    // Input channels
    let mut rcv_imu_sensor_array: [_; crate::N_IMU] = core::array::from_fn(|i| msg::IMU_SENSOR[i].receiver().unwrap());
    let mut rcv_start_gyr_calib = msg::CMD_START_GYR_CALIB.receiver().unwrap();

    // Output channels
    let snd_imu_config = msg::CFG_IMU_CONFIG.sender();
    let snd_gyr_calibrating = msg::GYR_CALIBRATING.sender();
    
    'infinite: loop {
        
        // Wait for the start signal
        snd_gyr_calibrating.send(false);
        if !rcv_start_gyr_calib.changed().await { continue 'infinite };
        snd_gyr_calibrating.send(true);
        defmt::info!("[GYR CALIB]: Starting bias calibration on {} gyroscopes", crate::N_IMU);

        // Array of calibrator instances
        let mut sensor_calibrator = [GyrCalibrator::<200>::new(); crate::N_IMU];
        
        let mut timeout_count = 0;
        let mut done_count = 0;

        let mut ticker = Ticker::every(Duration::from_hz(50));
        
        // Calibration loop
        'calibration: loop {

            ticker.next().await;

            // Iterate through all IMUs
            'iter_sensors: for (id, (rcv_imu, calibrator)) in rcv_imu_sensor_array.iter_mut().zip(sensor_calibrator.iter_mut()).enumerate() {

                // If the calibration has timed out or is complete, skip this sensor
                if calibrator.has_timeout() || calibrator.is_done() {
                    continue 'iter_sensors;
                }

                // Await IMU data, or detect time out
                match with_timeout(Duration::from_millis(50), rcv_imu.changed()).await {

                    // Insert IMU data into the calibrator
                    Ok((imu_data, _time)) => {
                        calibrator.collect(imu_data);
                        if calibrator.is_done() {
                            defmt::trace!("[GYR CALIB]: Gyr calibration complete for sensor {}", id);
                            done_count += 1;
                        }
                    },

                    // IMU Timed out
                    Err(_) => {
                        calibrator.set_timeout();
                        defmt::trace!("[GYR CALIB]: Gyr calibration timed out for sensor {}", id);
                        timeout_count += 1; 
                    },
                }

                // Check if calibration has been stopped by user
                if rcv_start_gyr_calib.try_changed() == Some(false) {
                    defmt::info!("[GYR CALIB]: Calibration process stopped by user");
                    continue 'infinite;
                }
            }

            // If all sensors have timed out or are done, break calibration loop
            if timeout_count + done_count == crate::N_IMU {
                break 'calibration;
            }
        }

        // Get existing calibration data or create new
        let mut calibrations = snd_imu_config.try_get().unwrap_or([None; crate::N_IMU]);

        // Publish result of each sensor if calibration was successful
        for (id, (opt_config, calibrator)) in calibrations.iter_mut().zip(sensor_calibrator.iter()).enumerate() {
            
            match (calibrator.acc_variance(), opt_config) {

                // If calibration was successful, update the calibration data
                (Some(var), Some(config)) if var < 0.002 => {
                    defmt::info!("[GYR CALIB]: Sensor {} calibration is valid: Var({})", id, var);
        
                    // Set calibration offset/bias for sensor
                    if let Some(gyr_cal) = config.gyr_cal.as_mut() {
                        gyr_cal.offset = calibrator.get_bias();
                    } else {
                        config.gyr_cal = Some(Calibration{
                            scale: Vector3::new(1.0, 1.0, 1.0),
                            offset: calibrator.get_bias(),
                        });
                    }
                },

                // If calibration was not successful, log the reason
                (Some(var), Some(_)) => defmt::error!("[GYR CALIB]: Sensor {} had too high variance: Var({:?})", id, var),
                (_, Some(_)) if calibrator.has_timeout() => defmt::error!("[GYR CALIB]: Sensor {} timed out.", id),
                _ => defmt::error!("[GYR CALIB]: Sensor {} could not be calibrated.", id),
            }
        }

        snd_imu_config.send(calibrations);

        // Ensure any recent request to calibrate is marked as seen
        let _ = rcv_start_gyr_calib.try_get();
    }
}

#[derive(Debug, Clone, Copy)]
struct GyrCalibrator<const N: usize> {
    gyr_bias_sum: Vector3<f32>,
    acc_buffer: SMatrix<f32, 3, N>,
    has_timed_out: bool,
    count: usize, 
}

impl <const N: usize> GyrCalibrator<N> {
    pub fn new() -> Self {
        Self {
            gyr_bias_sum: Vector3::zeros(),
            acc_buffer: SMatrix::zeros(),
            has_timed_out: false,
            count: 0,
        }
    }

    pub fn collect(&mut self, meas: ImuData6Dof) {
        self.gyr_bias_sum += meas.gyr;
        self.acc_buffer.set_column(self.count, &meas.acc);
        self.count += 1;
    }

    pub fn get_bias(&self) -> Vector3<f32> {
        self.gyr_bias_sum / self.count as f32
    }

    pub fn has_timeout(&self) -> bool {
        self.has_timed_out
    }

    pub fn set_timeout(&mut self) {
        self.has_timed_out = true;
    }

    pub fn acc_variance(&self) -> Option<f32> {
        if self.is_done() {
            Some(self.acc_buffer.column_variance().norm())
        } else {
            None
        }
    }

    pub fn is_done(&self) -> bool {
        self.count >= N
    }
}
