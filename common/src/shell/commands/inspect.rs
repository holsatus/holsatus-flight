use crate::{
    errors::adapter::embedded_io::EmbeddedIoError,
    shell::CLEAR_SCREEN,
    utils::u_types::{UBuffer, UFloat},
};
use embassy_time::Timer;
use embedded_cli::{
    arguments::{FromArgument, FromArgumentError},
    Command,
};
use embedded_io_async::{Read, Write};
use ufmt::uwriteln;

pub enum SensorKind {
    Attitude,
    Gyroscope,
    Accelerometer,
}

impl FromArgument<'_> for SensorKind {
    fn from_arg(arg: &'_ str) -> Result<Self, embedded_cli::arguments::FromArgumentError<'_>>
    where
        Self: Sized,
    {
        match arg {
            "att" | "attitude" | "orientation" => Ok(SensorKind::Attitude),
            "gyr" | "gyro" | "gyroscope" => Ok(SensorKind::Gyroscope),
            "acc" | "accel" | "accelerometer" => Ok(SensorKind::Accelerometer),
            _ => Err(FromArgumentError {
                value: arg,
                expected: "Not a valid sensor, try 'gyro', 'accel'.",
            }),
        }
    }
}

#[derive(Command)]
#[command(help_title = "Inspect commands")]
pub enum InspectCommand {
    /// View the current arming status
    Value {
        kind: SensorKind,

        /// The update frequency of the display
        #[arg(short = 'f', long)]
        frequency: Option<u8>,
    },
}

impl super::CommandHandler for InspectCommand {
    async fn handler(
        &self,
        mut serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> Result<(), EmbeddedIoError> {
        match self {
            InspectCommand::Value { kind, frequency } => {
                let mut maybe_ticker = super::periodic_ticker(&mut serial, *frequency).await?;

                loop {
                    if frequency.is_some() {
                        serial.write_all(CLEAR_SCREEN).await?;
                        serial.write_all(b"Press Ctrl-C to exit\n\r").await?;
                    }

                    match kind {
                        SensorKind::Attitude => {
                            serial.write_all(b"Estimated attitude data:\n\r").await?;
                            if let Some(att_est) = crate::signals::AHRS_ATTITUDE_Q.try_get() {
                                let mut buffer = UBuffer::<128>::new();
                                let rpy = att_est.euler_angles();
                                uwriteln!(buffer, "- x: {} rad", UFloat(rpy.0, 4))?;
                                uwriteln!(buffer, "- y: {} rad", UFloat(rpy.1, 4))?;
                                uwriteln!(buffer, "- z: {} rad", UFloat(rpy.2, 4))?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                        }
                        SensorKind::Gyroscope => {
                            serial.write_all(b"Raw gyroscope data:\n\r").await?;
                            if let Some(imu_data) = crate::signals::RAW_MULTI_IMU_DATA[0].try_get()
                            {
                                let mut buffer = UBuffer::<128>::new();
                                uwriteln!(buffer, "- x: {} rad/s", UFloat(imu_data.gyr[0], 4))?;
                                uwriteln!(buffer, "- y: {} rad/s", UFloat(imu_data.gyr[1], 4))?;
                                uwriteln!(buffer, "- z: {} rad/s", UFloat(imu_data.gyr[2], 4))?;
                                serial.write_all(buffer.bytes()).await?;
                            } else {
                                serial
                                    .write_all(
                                        b"Could not get raw gyroscope data at this time.\n\r",
                                    )
                                    .await?;
                            }
                            Timer::after_millis(5).await; // We should be careful to not flood the endpoint!
                            serial.write_all(b"Calibrated gyroscope data:\n\r").await?;
                            if let Some(imu_data) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get()
                            {
                                let mut buffer = UBuffer::<128>::new();
                                uwriteln!(buffer, "- x: {} rad/s", UFloat(imu_data.gyr[0], 4))?;
                                uwriteln!(buffer, "- y: {} rad/s", UFloat(imu_data.gyr[1], 4))?;
                                uwriteln!(buffer, "- z: {} rad/s", UFloat(imu_data.gyr[2], 4))?;
                                serial.write_all(buffer.bytes()).await?;
                            } else {
                                serial.write_all(b"Could not get calibrated gyroscope data at this time.\n\r").await?;
                            }
                        }
                        SensorKind::Accelerometer => {
                            serial.write_all(b"Raw accelerometer data:\n\r").await?;
                            if let Some(imu_data) = crate::signals::RAW_MULTI_IMU_DATA[0].try_get()
                            {
                                let mut buffer = UBuffer::<128>::new();
                                uwriteln!(buffer, "- x: {} m/s^2", UFloat(imu_data.acc[0], 4))?;
                                uwriteln!(buffer, "- y: {} m/s^2", UFloat(imu_data.acc[1], 4))?;
                                uwriteln!(buffer, "- z: {} m/s^2", UFloat(imu_data.acc[2], 4))?;
                                serial.write_all(buffer.bytes()).await?;
                            } else {
                                serial
                                    .write_all(
                                        b"Could not get raw accelerometer data at this time.\n\r",
                                    )
                                    .await?;
                            }
                            Timer::after_millis(5).await; // We should be careful to not flood the endpoint!
                            serial
                                .write_all(b"Calibrated accelerometer data:\n\r")
                                .await?;
                            if let Some(imu_data) = crate::signals::CAL_MULTI_IMU_DATA[0].try_get()
                            {
                                let mut buffer = UBuffer::<128>::new();
                                uwriteln!(buffer, "- x: {} m/s^2", UFloat(imu_data.acc[0], 4))?;
                                uwriteln!(buffer, "- y: {} m/s^2", UFloat(imu_data.acc[1], 4))?;
                                uwriteln!(buffer, "- z: {} m/s^2", UFloat(imu_data.acc[2], 4))?;
                                serial.write_all(buffer.bytes()).await?;
                            } else {
                                serial.write_all(b"Could not get calibrated accelerometer data at this time.\n\r").await?;
                            }
                        }
                    }

                    serial.flush().await?;

                    if super::wait_next_or_ctrl_c(&mut serial, &mut maybe_ticker).await? {
                        break;
                    }
                }
            }
        }

        Ok(())
    }
}
