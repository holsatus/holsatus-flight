use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};
use embedded_cli::Command;
use embedded_io::ErrorKind;
use embedded_io_async::{Read, Write};
use ufmt::uwrite;

use crate::shell::{UBuffer, INTERRUPT};


#[derive(Command)]
#[command(help_title = "System commands")]
pub enum SystemCommand {
    /// Reboot the vehicle's FMU
    Reboot,

    /// Show the system uptime
    Uptime {

        /// The update frequency of the uptime display
        #[arg(short = 'f', long)]
        frequency: Option<u8>
    },
}

impl super::CommandHandler for SystemCommand {
    async fn handler(&self, mut serial: impl Read<Error = ErrorKind> + Write<Error = ErrorKind>) -> Result<(), ErrorKind> {
        match self {
            SystemCommand::Reboot => {
                serial.write_all(b"Rebooting...\n").await?;
            },
            SystemCommand::Uptime { frequency } => {

                // Configure ticker if a frequency is provided
                let mut maybe_ticker = frequency.map(|f|
                    Ticker::every(Duration::from_hz(f as u64))
                );

                loop {
                    let millis = embassy_time::Instant::now().as_millis();
                    let mut buffer = UBuffer::<32>::new();
                    uwrite!(buffer, "Uptime: {} ms\n\r", millis)?;
                    serial.write_all(&buffer.inner).await?;
                    serial.flush().await?;

                    // Maybe we can isolate this functionality into a helper function
                    match &mut maybe_ticker {
                        None => break,
                        Some(ticker) => {
                            let mut bytes = [0u8];
                            if let Either::First(res) = select(
                                serial.read(&mut bytes[..]),
                                ticker.next()).await 
                            {
                                match res {
                                    Ok(_) => if bytes[0] == *INTERRUPT { break },
                                    Err(e) => return Err(e),
                                }
                            }
                        },
                    }
                }
            },
        }

        Ok(())
    }
}
