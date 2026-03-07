use crate::errors::adapter::embedded_io::EmbeddedIoError;
use crate::tasks::motor_test::{MotorTest, SIGNAL};
use embedded_cli::Command;
use embedded_io_async::{Read, Write};

#[derive(Command)]
#[command(help_title = "Motor test commands")]
pub enum MotorCommand {
    /// Stop ongoing motor test output
    Stop,

    /// Run startup-like motor calibration profile (high then low)
    Calibrate {
        /// High throttle ratio in [0.0, 1.0]
        #[arg(long)]
        high: Option<f32>,

        /// Duration for high phase
        #[arg(long)]
        high_ms: Option<u16>,

        /// Duration for low phase
        #[arg(long)]
        low_ms: Option<u16>,
    },

    /// Run stepped motor test profile
    Step {
        /// Warmup duration per step cycle
        #[arg(long)]
        warmup_ms: Option<u16>,

        /// Hold duration per step level
        #[arg(long)]
        step_ms: Option<u16>,

        /// Final ratio in [0.0, 1.0]
        #[arg(long)]
        end: Option<f32>,
    },

    /// Drive one motor channel at fixed ratio
    Single {
        /// Motor index [0..3]
        id: u8,

        /// Ratio in [0.0, 1.0]
        speed: f32,

        /// Duration
        #[arg(long)]
        ms: Option<u16>,
    },
}

impl super::CommandHandler for MotorCommand {
    async fn handler(
        &self,
        mut serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> Result<(), EmbeddedIoError> {
        match self {
            MotorCommand::Stop => {
                SIGNAL.signal(MotorTest::Stop);
                serial
                    .write_all(b"[info] motor test stopped (arm state unchanged)\n\r")
                    .await?;
            }
            MotorCommand::Calibrate {
                high,
                high_ms,
                low_ms,
            } => {
                let high = high.unwrap_or(0.80).clamp(0.0, 1.0);
                let high_ms = high_ms.unwrap_or(1500);
                let low_ms = low_ms.unwrap_or(4500);

                SIGNAL.signal(MotorTest::Calibrate {
                    high,
                    millis_high: high_ms,
                    millis_low: low_ms,
                });

                serial
                    .write_all(
                        b"[info] queued motor calibration profile (requires armed motors)\n\r",
                    )
                    .await?;
            }
            MotorCommand::Step {
                warmup_ms,
                step_ms,
                end,
            } => {
                let warmup_ms = warmup_ms.unwrap_or(1200);
                let step_ms = step_ms.unwrap_or(1500);
                let end = end.unwrap_or(0.30).clamp(0.0, 1.0);

                SIGNAL.signal(MotorTest::Step {
                    millis_warmup: warmup_ms,
                    millis_step: step_ms,
                    end,
                });

                serial
                    .write_all(b"[info] queued motor step profile (requires armed motors)\n\r")
                    .await?;
            }
            MotorCommand::Single { id, speed, ms } => {
                if *id > 3 {
                    serial
                        .write_all(b"[warn] motor id must be in range 0..3\n\r")
                        .await?;
                    return Ok(());
                }

                let speed = speed.clamp(0.0, 1.0);
                let ms = ms.unwrap_or(2000);

                SIGNAL.signal(MotorTest::Single {
                    motor: *id,
                    speed,
                    millis: ms,
                });

                serial
                    .write_all(b"[info] queued single-motor profile (requires armed motors)\n\r")
                    .await?;
            }
        }

        Ok(())
    }
}
