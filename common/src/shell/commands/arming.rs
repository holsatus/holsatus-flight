use crate::{
    errors::adapter::embedded_io::EmbeddedIoError,
    shell::CLEAR_SCREEN,
    tasks::commander::{
        message::{ArmDisarm, Origin, Request, Response},
        PROCEDURE,
    },
    types::status::ArmingBlocker,
    utils::u_types::UBuffer,
};
use embedded_cli::Command;
use embedded_io_async::{Read, Write};
use ufmt::{uWrite, uwrite};

#[derive(Command)]
#[command(help_title = "Arming commands")]
pub enum ArmingCommand {
    /// Request arming through commander checks
    Arm,

    /// Request arming and bypass checks
    ArmForce,

    /// Request disarming through commander checks
    Disarm,

    /// Request immediate disarm and bypass checks
    DisarmForce,

    /// View the current arming status
    Status,

    /// View the arming safety flags
    Flags {
        /// The update frequency of the uptime display
        #[arg(short = 'f', long)]
        frequency: Option<u8>,
    },
}

impl super::CommandHandler for ArmingCommand {
    async fn handler(
        &self,
        mut serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> Result<(), EmbeddedIoError> {
        match self {
            ArmingCommand::Arm => {
                let res = PROCEDURE
                    .request(Request {
                        command: ArmDisarm {
                            arm: true,
                            force: false,
                        }
                        .into(),
                        origin: Origin::CommandLine,
                    })
                    .await;

                match res {
                    Some(Response::Accepted) | Some(Response::Unchanged) => {
                        serial.write_all(b"Arming request accepted\n\r").await?;
                    }
                    Some(other) => {
                        let _ = other;
                        serial.write_all(b"Arming request rejected\n\r").await?;
                    }
                    None => {
                        serial
                            .write_all(b"No response from commander\n\r")
                            .await?;
                    }
                }
            }
            ArmingCommand::ArmForce => {
                let res = PROCEDURE
                    .request(Request {
                        command: ArmDisarm {
                            arm: true,
                            force: true,
                        }
                        .into(),
                        origin: Origin::CommandLine,
                    })
                    .await;

                match res {
                    Some(Response::Accepted) | Some(Response::Unchanged) => {
                        serial.write_all(b"Force-arm request accepted\n\r").await?;
                    }
                    Some(other) => {
                        let _ = other;
                        serial.write_all(b"Force-arm request rejected\n\r").await?;
                    }
                    None => {
                        serial
                            .write_all(b"No response from commander\n\r")
                            .await?;
                    }
                }
            }
            ArmingCommand::Disarm => {
                let res = PROCEDURE
                    .request(Request {
                        command: ArmDisarm {
                            arm: false,
                            force: false,
                        }
                        .into(),
                        origin: Origin::CommandLine,
                    })
                    .await;

                match res {
                    Some(Response::Accepted) | Some(Response::Unchanged) => {
                        serial.write_all(b"Disarm request accepted\n\r").await?;
                    }
                    Some(other) => {
                        let _ = other;
                        serial.write_all(b"Disarm request rejected\n\r").await?;
                    }
                    None => {
                        serial
                            .write_all(b"No response from commander\n\r")
                            .await?;
                    }
                }
            }
            ArmingCommand::DisarmForce => {
                let res = PROCEDURE
                    .request(Request {
                        command: ArmDisarm {
                            arm: false,
                            force: true,
                        }
                        .into(),
                        origin: Origin::CommandLine,
                    })
                    .await;

                match res {
                    Some(Response::Accepted) | Some(Response::Unchanged) => {
                        serial
                            .write_all(b"Force-disarm request accepted\n\r")
                            .await?;
                    }
                    Some(other) => {
                        let _ = other;
                        serial
                            .write_all(b"Force-disarm request rejected\n\r")
                            .await?;
                    }
                    None => {
                        serial
                            .write_all(b"No response from commander\n\r")
                            .await?;
                    }
                }
            }
            ArmingCommand::Status => {
                serial
                    .write_all(b"Fetching arming status is currently not available\n\r")
                    .await?;
            }
            ArmingCommand::Flags { frequency } => {
                let mut maybe_ticker = super::periodic_ticker(&mut serial, *frequency).await?;

                loop {
                    let Some(blocker) = crate::signals::ARMING_BLOCKER.try_get() else {
                        serial
                            .write_all(b"ARMING_BLOCKER value not available\n\r")
                            .await?;
                        serial.flush().await?;
                        break;
                    };

                    if frequency.is_some() {
                        serial.write_all(CLEAR_SCREEN).await?;
                        serial.write_all(b"Press Ctrl-C to exit\n\r").await?;
                    }
                    serial.write_all(b"ArmingBlocker status\n\r").await?;

                    // Hopefully the compiler is smart enough to optimize this,
                    // the iter_names method is const, so the longest should
                    // always be the same.
                    let longest = ArmingBlocker::all()
                        .iter_names()
                        .fold(0, |acc, (name, _)| acc.max(name.len()));
                    for (name, flag) in ArmingBlocker::all().iter_names() {
                        let mut buffer = UBuffer::<32>::new();
                        // Write flag name ane spacing
                        uwrite!(buffer, "{}", name)?;
                        for _ in 0..longest.saturating_sub(name.len()) {
                            buffer.write_char(' ')?;
                        }

                        // Write flag status
                        if blocker.intersects(flag) {
                            buffer.write_str(" BAD\n\r")?;
                        } else {
                            buffer.write_str(" ---\n\r")?;
                        }

                        // Write buffer to serial
                        serial.write_all(buffer.bytes()).await?;
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
