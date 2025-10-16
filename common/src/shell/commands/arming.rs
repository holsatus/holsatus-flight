use crate::{
    errors::adapter::embedded_io::EmbeddedIoError,
    shell::{CLEAR_SCREEN, INTERRUPT},
    types::status::ArmingBlocker,
    utils::u_types::UBuffer,
};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use embedded_cli::Command;
use embedded_io_async::{Read, Write};
use ufmt::{uWrite, uwrite};

#[derive(Command)]
#[command(help_title = "Arming commands")]
pub enum ArmingCommand {
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
            ArmingCommand::Status => {
                serial
                    .write_all(b"Fetching arming status is currently not available\n\r")
                    .await?;
            }
            ArmingCommand::Flags { frequency } => {
                let mut maybe_ticker =
                    frequency.map(|f| Ticker::every(Duration::from_hz(f as u64)));

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

                    // TODO - Maybe we can isolate this functionality into a
                    // helper function
                    match &mut maybe_ticker {
                        None => break,
                        Some(ticker) => {
                            let mut bytes = [0u8];
                            if let Either::First(res) =
                                select(serial.read(&mut bytes[..]), ticker.next()).await
                            {
                                match res {
                                    Ok(_) => {
                                        if bytes[0] == *INTERRUPT {
                                            break;
                                        }
                                    }
                                    Err(e) => return Err(e),
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(())
    }
}
