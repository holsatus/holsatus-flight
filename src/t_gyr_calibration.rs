use embassy_time::{with_timeout, Duration, Instant, Ticker};
use nalgebra::{SMatrix, Vector3};

use crate::{config::Calibration, messaging as msg, sensors::imu::types::ImuData6Dof};

const DEFAULT_MAX_VAR: f32 = 0.002;
const DEFAULT_DURATION: u8 = 5;
const MAX_CALIBRATION_DURATION: u8 = 120;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyrCalCommand {
    Start(CalGyrConfig),
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalGyrConfig {

    // ID of sensor to calibrate, or `None` for all availabe sensors.
    pub sensor: Option<u8>,

    /// Duration of calibration in seconds, or `None` for default duration of 5 seconds.
    pub duration: Option<u8>,

    /// Maximum variance of the accelerometer data during calibration, or `None` for default value of 0.002.
    pub max_var: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyrCalStatus {
    Idle,
    Done([Option<f32>; crate::N_IMU]),
    Collecting{
        sensors: Option<u8>,
        timeleft_ms: u32
    },
}

/// Routine to calibrate gyroscopes, by calculating their bias.
#[embassy_executor::task]
pub async fn gyr_calibration() -> ! {

    // Input channels
    let mut rcv_imu_sensor_array: [_; crate::N_IMU] = core::array::from_fn(|i| msg::IMU_SENSOR[i].receiver().unwrap());
    let mut rcv_start_gyr_calib = msg::CMD_GYR_CALIB.receiver().unwrap();

    // Output channels
    let snd_imu_config = msg::CFG_IMU_CONFIG.sender();
    let snd_gyr_calibrating = msg::GYR_CALIBRATING.sender();
    let snd_gyr_cal_status = msg::CMD_GYR_STATUS.sender();

    snd_gyr_cal_status.send(GyrCalStatus::Idle);
    
    'infinite: loop {
        
        // Wait for the start signal TODO migrate this signal to just be part of the GyrCalStatus
        snd_gyr_calibrating.send(false);

        let command = rcv_start_gyr_calib.changed().await;

        let cfg = match command {
            GyrCalCommand::Start(cfg) => cfg,
            GyrCalCommand::Stop => {
                defmt::info!("[GYR CALIB]: Calibration process not currently running, ignoring stop command.");
                continue 'infinite;
            },
        };
        snd_gyr_calibrating.send(true);

        let max_var = cfg.max_var.unwrap_or(DEFAULT_MAX_VAR);

        match cfg.sensor {
            Some(sensor) if sensor >= crate::N_IMU as u8 => {
                defmt::info!("[GYR CALIB]: Firmware does not support sensor id of {} ", sensor);
                continue 'infinite;
            },
            Some(sensor) => defmt::info!("[GYR CALIB]: Starting bias calibration on sensor {}, max variance of {} ", sensor, max_var),
            None => defmt::info!("[GYR CALIB]: Starting bias calibration on {} gyroscopes, max variance of {} ", crate::N_IMU, max_var),
        }

        // Array of calibrator instances
        let mut sensor_calibrator = [GyrCalibrator::<200>::new(); crate::N_IMU];
        
        let mut timeout_count = 0;
        let mut done_count = 0;

        let duration_s = cfg.duration.unwrap_or(DEFAULT_DURATION).min(MAX_CALIBRATION_DURATION);
        let end_time = Instant::now() + Duration::from_secs(duration_s as u64);
        let mut ticker = Ticker::every(Duration::from_hz((200/duration_s) as u64));
        
        // Calibration loop
        'calibration: loop {

            snd_gyr_cal_status.send(GyrCalStatus::Collecting{
                sensors: cfg.sensor, 

                // Calculate time left as ms
                timeleft_ms: 
                end_time.checked_duration_since(Instant::now())
                .unwrap_or(Duration::from_ticks(0))
                .as_millis() as u32
            });

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
                    Ok(imu_data) => {
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
                        if cfg.sensor == Some(id as u8) {
                            break 'calibration;
                        }
                    },
                }

                // Check if calibration has been stopped by user
                if rcv_start_gyr_calib.try_changed() == Some(GyrCalCommand::Stop) {
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
        let mut variances = [None; crate::N_IMU];

        // Publish result of each sensor if calibration was successful
        'sensor_iter: for (id, (opt_config, calibrator)) in calibrations.iter_mut().zip(sensor_calibrator.iter()).enumerate() {
            if cfg.sensor.is_some() && cfg.sensor != Some(id as u8) {
                continue 'sensor_iter;
            }
            match (calibrator.acc_variance(), opt_config) {

                // If calibration was successful, update the calibration data
                (Some(var), Some(config)) if var < max_var => {

                    variances[id] = Some(var);
                    
                    // Set calibration offset/bias for sensor
                    if let Some(gyr_cal) = config.gyr_cal.as_mut() {
                        gyr_cal.offset = calibrator.get_bias();
                    } else {
                        config.gyr_cal = Some(Calibration{
                            scale: Vector3::new(1.0, 1.0, 1.0),
                            offset: calibrator.get_bias(),
                        });
                    }
                    let bias: [f32; 3] = calibrator.get_bias().into();
                    defmt::info!("[GYR CALIB]: Sensor {} calibration is valid: Var({}) of {:?}", id, var, bias);
                },

                // If calibration was not successful, log the reason
                (Some(var), Some(_)) => defmt::error!("[GYR CALIB]: Sensor {} had too high variance: Var({:?})", id, var),
                (_, Some(_)) if calibrator.has_timeout() => defmt::error!("[GYR CALIB]: Sensor {} timed out.", id),
                _ => defmt::error!("[GYR CALIB]: Sensor {} could not be calibrated.", id),
            }
        }

        snd_imu_config.send(calibrations);
        snd_gyr_cal_status.send(GyrCalStatus::Done(variances));

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

// TODO, each calibrator could calculate the variance over a smaller number of sampels,
// and average the result. This enables us to use a much smaller buffer, while still
// getting a good estimate of the variance.

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
