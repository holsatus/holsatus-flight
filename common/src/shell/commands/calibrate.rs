use embassy_time::{with_timeout, Duration, Timer};
use embedded_cli::Command;
use embedded_io_async::{Read, Write};
use ufmt::{uDisplay, uwrite};

use crate::{
    calibration::mag_routine::MagCalState,
    errors::adapter::embedded_io::EmbeddedIoError,
    tasks::{
        calibrator::Sensor,
        commander::{message::*, PROCEDURE},
    },
    utils::u_types::UBuffer,
};

#[allow(unused)] // TODO until implementation is fixed
#[derive(Command)]
#[command(help_title = "Calibration commands")]
pub enum CalibrateCommand {
    /// Start the accelerometer calibration routine
    Acc {
        /// The ID of the sensor to calibrate
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Maximum permitted variance in the calibration data
        #[arg(short = 'v', long)]
        max_variance: Option<f32>,

        /// Maximum number of allowed dropped samples
        #[arg(short = 'd', long)]
        max_dropped: Option<usize>,
    },

    /// Start the gyroscope calibration routine
    Gyr {
        /// The ID of the sensor to calibrate
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Maximum permitted variance in the calibration data
        #[arg(short = 'v', long)]
        max_variance: Option<f32>,

        /// Maximum number of allowed dropped samples
        #[arg(short = 'd', long)]
        max_dropped: Option<usize>,

        /// Duration of the calibration routine
        #[arg(short = 't', long)]
        duration: Option<u8>,
    },

    /// Start the magnetometer calibration routine
    Mag {
        /// The ID of the sensor to calibrate
        #[arg(short = 's', long)]
        sensor: Option<u8>,

        /// Maximum number of allowed dropped samples
        #[arg(short = 'd', long)]
        max_dropped: Option<usize>,

        /// Prescalar to apply to the magnetometer data
        #[arg(short = 'p', long)]
        pre_scalar: Option<f32>,
    },

    /// Start the barometer calibration routine
    Baro,

    /// List the calibration status of all sensors
    Info,
}

impl super::CommandHandler for CalibrateCommand {
    async fn handler(
        &self,
        mut serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> Result<(), EmbeddedIoError> {
        match self {
            CalibrateCommand::Acc {
                sensor: _,
                max_variance: _,
                max_dropped: _,
            } => {
                const _DEFAULT_MAX_VAR: f32 = 0.1;
                const _DEFAULT_MAX_DROPPED: usize = 5;

                serial
                    .write_all(b"Requesting accelerometer calibration\n\r")
                    .await?;
                PROCEDURE
                    .request(Request {
                        command: DoCalibration {
                            sensor_id: None,
                            sensor_type: SensorType::Accelerometer,
                        }
                        .into(),
                        origin: Origin::Unspecified,
                    })
                    .await;

                // Give the calibrator some time to start
                Timer::after_millis(10).await;
                let mut receiver = crate::signals::CALIBRATOR_STATE.receiver();

                loop {
                    let Ok(calibration_status) =
                        with_timeout(Duration::from_secs(2), receiver.changed()).await
                    else {
                        serial
                            .write_all(b"Calibration failed to respond\n\r")
                            .await?;
                        break;
                    };

                    match calibration_status {
                        crate::tasks::calibrator::CalibratorState::Idle => {
                            serial.write_all(b"Calibration idle\n\r").await?;
                            break;
                        }
                        crate::tasks::calibrator::CalibratorState::Calibrating(Sensor::Acc) => {
                            let mut buffer = UBuffer::<32>::new();
                            uwrite!(buffer, "Calibrating sensor\n\r")?;
                            serial.write_all(buffer.bytes()).await?;
                        }
                        _ => {
                            serial
                                .write_all(
                                    b"error: Calibration of different sensor in progress\n\r",
                                )
                                .await?;
                            break;
                        }
                    }
                }
            }
            CalibrateCommand::Gyr {
                sensor: _,
                max_variance: _,
                max_dropped: _,
                duration: _,
            } => {
                const _DEFAULT_MAX_VAR: f32 = 0.01;
                const _DEFAULT_MAX_DROPPED: usize = 5;
                const _DEFAULT_DURATION: u8 = 5;

                serial
                    .write_all(b"Requesting gyroscope calibration\n\r")
                    .await?;
                PROCEDURE
                    .request(Request {
                        command: DoCalibration {
                            sensor_id: None,
                            sensor_type: SensorType::Gyroscope,
                        }
                        .into(),
                        origin: Origin::Unspecified,
                    })
                    .await;

                // Give the calibrator some time to start
                Timer::after_millis(10).await;
                let mut receiver = crate::signals::CALIBRATOR_STATE.receiver();

                loop {
                    let Ok(calibration_status) =
                        with_timeout(Duration::from_secs(2), receiver.changed()).await
                    else {
                        serial
                            .write_all(b"Calibration failed to respond\n\r")
                            .await?;
                        break;
                    };

                    match calibration_status {
                        crate::tasks::calibrator::CalibratorState::Idle => {
                            serial.write_all(b"Calibration idle\n\r").await?;
                            break;
                        }
                        crate::tasks::calibrator::CalibratorState::Calibrating(Sensor::Gyr) => {
                            let mut buffer = UBuffer::<32>::new();
                            uwrite!(buffer, "Calibrating sensor\n\r")?;
                            serial.write_all(buffer.bytes()).await?;
                        }
                        _ => {
                            serial
                                .write_all(
                                    b"error: Calibration of different sensor in progress\n\r",
                                )
                                .await?;
                            break;
                        }
                    }
                }
            }
            CalibrateCommand::Mag {
                sensor: _,
                max_dropped: _,
                pre_scalar: _,
            } => {
                const _DEFAULT_MAX_VAR: f32 = 0.01;
                const _DEFAULT_MAX_DROPPED: usize = 5;
                const _DEFAULT_PRESCALAR: f32 = 50.;
                const _DEFAULT_DURATION: u8 = 5;

                serial
                    .write_all(b"Requesting gyroscope calibration\n\r")
                    .await?;
                PROCEDURE
                    .request(Request {
                        command: DoCalibration {
                            sensor_id: None,
                            sensor_type: SensorType::Magnetometer,
                        }
                        .into(),
                        origin: Origin::Unspecified,
                    })
                    .await;

                // Give the calibrator some time to start
                Timer::after_millis(10).await;
                let mut receiver = crate::signals::CALIBRATOR_STATE.receiver();

                loop {
                    let Ok(calibration_status) =
                        with_timeout(Duration::from_secs(2), receiver.changed()).await
                    else {
                        serial
                            .write_all(b"Calibration failed to respond\n\r")
                            .await?;
                        break;
                    };

                    match calibration_status {
                        crate::tasks::calibrator::CalibratorState::Idle => {
                            serial.write_all(b"Calibration idle\n\r").await?;
                        }
                        crate::tasks::calibrator::CalibratorState::Calibrating(Sensor::Mag(
                            state,
                        )) => match state {
                            MagCalState::Initialized => {
                                let mut buffer = UBuffer::<32>::new();
                                uwrite!(buffer, "Calibrating sensor\n\r")?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                            MagCalState::Collecting(state) => {
                                let mut buffer = UBuffer::<64>::new();
                                uwrite!(
                                    buffer,
                                    "Status: {}, sample: [{}, {}, {}]\n\r",
                                    Float(state.mean_distance, 5),
                                    Float(state.sample[0], 2),
                                    Float(state.sample[1], 2),
                                    Float(state.sample[2], 2)
                                )?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                            MagCalState::DoneSuccess(_) => {
                                let mut buffer = UBuffer::<32>::new();
                                uwrite!(buffer, "Successfully calibrated magnetometer!\n\r")?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                            MagCalState::DoneFailed(_) => {
                                let mut buffer = UBuffer::<32>::new();
                                uwrite!(buffer, "Failed to calibrated magnetomter\n\r")?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                        },
                        _ => {
                            serial
                                .write_all(
                                    b"error: Calibration of different sensor in progress\n\r",
                                )
                                .await?;
                            break;
                        }
                    }
                }
            }
            CalibrateCommand::Baro => {
                serial.write_all(b"Calibrating barometer\n\r").await?;
            }
            CalibrateCommand::Info => {
                serial.write_all(b"Calibration info\n\r").await?;
            }
        }

        Ok(())
    }
}

struct Float(f32, u8);

fn float_to_int_f32(orginal: f32, precision: u8) -> (i32, u32) {
    let prec = match precision {
        1 => 1e1,
        2 => 1e2,
        3 => 1e3,
        4 => 1e4,
        5 => 1e5,
        _ => 0.0,
    };
    let base = orginal as i32;
    let decimal = ((orginal - (base as f32)) * prec) as u32;
    (base, decimal)
}

impl uDisplay for Float {
    fn fmt<W>(&self, fmt: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        let (base, decimal) = float_to_int_f32(self.0, self.1);
        uwrite!(fmt, "{}.{}", base, decimal)
    }
}
