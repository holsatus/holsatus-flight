use core::ops::DerefMut;

use commands::CommandHandler;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Timer;
use embedded_cli::{cli::CliBuilder, Command};
use embedded_io::{ErrorType, Write as SyncWrite};
use embedded_io_async::{Read, Write};

use crate::{errors::adapter::embedded_io::EmbeddedIoError, serial::IoStream};

mod commands;

/// Wrapper to allow the CLI to own a synchronous writer. This is a bit of a
/// hack since embedded-cli does not support async (and takes ownership of the
/// writer..)
struct SyncWriter<'a, W: Write<Error = E>, E: embedded_io::Error> {
    writer: &'a Mutex<NoopRawMutex, W>,
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> SyncWriter<'a, W, E> {
    pub fn new(writer: &'a Mutex<NoopRawMutex, W>) -> Self {
        Self { writer }
    }
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> ErrorType for SyncWriter<'a, W, E> {
    type Error = E;
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> SyncWrite for SyncWriter<'a, W, E> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Avoid panicking if another path still holds the writer briefly.
        // Blocking here is preferable to crashing the shell task.
        let mut writer = embassy_futures::block_on(self.writer.lock());
        embassy_futures::block_on(writer.write(buf))
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // See `write`: serialize access instead of panic-on-contention.
        let mut writer = embassy_futures::block_on(self.writer.lock());
        embassy_futures::block_on(writer.flush())
    }
}

#[embassy_executor::task]
pub async fn main(serial_id: &'static str) -> ! {
    let mut serial = crate::serial::claim(serial_id).unwrap();
    let mut rcv_usb_connected = crate::signals::USB_CONNECTED.receiver();
    loop {
        // Wait for the connection to be active, otherwise wait here
        let con = rcv_usb_connected
            .get_and(|&connected| connected == true)
            .await;
        info!("[cli] usb connection: {:?}", con);

        if let Err(error) = run_cli(&mut serial).await {
            match error {
                EmbeddedIoError::NotConnected | EmbeddedIoError::TimedOut => {
                    info!("[cli] USB session ended: {:?}", error);
                    Timer::after_millis(100).await;
                }
                _ => error!("[cli] Error on serial device: {:?}", error),
            }
        }
    }
}

pub async fn run_cli(serial: &mut IoStream) -> Result<(), EmbeddedIoError> {
    info!("[cli] Starting CLI runner");

    let mut command_buffer = [0u8; 128];
    let mut history_buffer = [0u8; 128];

    let mutexed_serial = Mutex::new(serial);

    // Give com software time to connect
    Timer::after_secs(1).await;

    {
        let mut m_serial = mutexed_serial.lock().await;
        m_serial.write_all(CLEAR_SCREEN).await?;
        m_serial.write_all(HOLSATUS_GRAPHIC).await?;
        m_serial.write_all(CMD_PROMPT.as_bytes()).await?;
    }

    let mut sync_writer = SyncWriter::new(&mutexed_serial);
    let mut cli = CliBuilder::default()
        .writer(&mut sync_writer)
        .command_buffer(command_buffer.as_mut_slice())
        .history_buffer(history_buffer.as_mut_slice())
        .prompt(CMD_PROMPT)
        .build()?;

    let mut buffer = [0u8; 64];
    let mut escape_state = EscapeState::None;
    loop {
        let n = {
            let mut m_serial = mutexed_serial.lock().await;
            let Ok(n) = m_serial.read(&mut buffer).await else {
                Timer::after_millis(100).await;
                continue;
            };
            n
        };

        if n == 0 {
            // Some backends report "no data" as an empty read.
            // Yield briefly to avoid pegging the executor.
            Timer::after_millis(1).await;
            continue;
        }

        trace!("Read {} bytes from serial port: {:?}", n, &buffer[..n]);

        for byte in buffer.iter().take(n) {
            let normalized_byte = match *byte {
                0x7F => 0x08, // DEL -> Backspace
                b'\n' => b'\r', // normalize newline
                value => value,
            };

            // Ignore ANSI escape/control sequences often emitted by terminals
            // on startup and during line editing.
            match escape_state {
                EscapeState::None => {
                    if normalized_byte == 0x1B {
                        escape_state = EscapeState::Esc;
                        continue;
                    }
                }
                EscapeState::Esc => {
                    escape_state = match normalized_byte {
                        b'[' => EscapeState::Csi,
                        b']' => EscapeState::Osc,
                        _ => EscapeState::None,
                    };
                    continue;
                }
                EscapeState::Csi => {
                    if (0x40..=0x7E).contains(&normalized_byte) {
                        escape_state = EscapeState::None;
                    }
                    continue;
                }
                EscapeState::Osc => {
                    escape_state = match normalized_byte {
                        0x07 => EscapeState::None,
                        0x1B => EscapeState::OscEsc,
                        _ => EscapeState::Osc,
                    };
                    continue;
                }
                EscapeState::OscEsc => {
                    escape_state = if normalized_byte == b'\\' {
                        EscapeState::None
                    } else {
                        EscapeState::Osc
                    };
                    continue;
                }
            }

            let is_allowed = normalized_byte == b'\r'
                || normalized_byte == 0x08
                || normalized_byte == b'\t'
                || normalized_byte.is_ascii_graphic()
                || normalized_byte == b' ';

            if !is_allowed {
                continue;
            }

            // Process the byte from the serial port
            let mut parsed_command = None;
            let process_result = {
                let mut processor = Base::processor(|_, command| {
                    parsed_command = Some(command);
                    Ok(())
                });
                cli.process_byte::<Base, _>(normalized_byte, &mut processor)
            };

            if let Err(error) = process_result {
                error!("[cli] parse error: {:?}", error);

                command_buffer.fill(0);
                history_buffer.fill(0);
                cli = CliBuilder::default()
                    .writer(&mut sync_writer)
                    .command_buffer(command_buffer.as_mut_slice())
                    .history_buffer(history_buffer.as_mut_slice())
                    .prompt(CMD_PROMPT)
                    .build()?;

                let mut m_serial = mutexed_serial.lock().await;
                let serial = m_serial.deref_mut();
                let _ = serial.write_all(CLEAR_LINE).await;
                let _ = serial.write_all(b"\r").await;
                let _ = serial.write_all(b"[error] Parse error; input reset\n\r").await;
                let _ = serial.write_all(CMD_PROMPT.as_bytes()).await;
                let _ = serial.flush().await;
                continue;
            }

            if let Some(command) = parsed_command {
                // Lock the serial port for writing (with async!)
                let mut m_serial = mutexed_serial.lock().await;
                let mut serial = m_serial.deref_mut();

                // Remove the prompt and return cursor to start of line
                serial.write_all(CLEAR_LINE).await?;
                serial.write_all(b"\r").await?;

                let command_result = match command {
                    Base::Cal { cmd } => cmd.handler(&mut serial).await,
                    Base::Sys { cmd } => cmd.handler(&mut serial).await,
                    #[cfg(feature = "mavlink")]
                    Base::Mavlink { cmd } => cmd.handler(&mut serial).await,
                    Base::Arming { cmd } => cmd.handler(&mut serial).await,
                    Base::Motor { cmd } => cmd.handler(&mut serial).await,
                    Base::Param { cmd } => cmd.handler(&mut serial).await,
                    Base::Inspect { cmd } => cmd.handler(&mut serial).await,

                    Base::Clear => serial.write_all(CLEAR_SCREEN).await,
                    Base::Splash => serial.write_all(HOLSATUS_GRAPHIC).await,
                };

                if let Err(error) = command_result {
                    match error {
                        EmbeddedIoError::NotConnected | EmbeddedIoError::TimedOut => {
                            return Err(error);
                        }
                        _ => {
                            error!("[cli] command handler error: {:?}", error);
                            let _ = serial.write_all(b"[error] Command failed\n\r").await;
                        }
                    }
                }

                // At the end of parsed command, show the prompt
                serial.write_all(b"\n\r").await?;
                serial.write_all(CMD_PROMPT.as_bytes()).await?;
                serial.flush().await?;
            }
        }
    }
}

enum EscapeState {
    None,
    Esc,
    Csi,
    Osc,
    OscEsc,
}

const CLEAR_SCREEN: &[u8] = b"\x1B[2J";
const CLEAR_LINE: &[u8] = b"\x1B[2K";
const CMD_PROMPT: &str = "$ ";
const HOLSATUS_GRAPHIC: &[u8] = b"\x1B[32m
\r   _   _       _           _
\r  | | | |     | |         | |
\r  | |_| | ___ | |___  __ _| |_ _   _ ___ 
\r  |  _  |/ _ \\| / __|/ _` | __| | | / __|
\r  | | | | (_) | \\__ \\ (_| | |_| |_| \\__ \\
\r  \\_| |_/\\___/|_|___/\\__,_|\\__|\\__,_|___/
\r
\x1B[0m";

#[derive(Command)]
enum Base {
    /// Clear the shell output
    Clear,

    /// Show the Holsatus graphic
    Splash,

    /// Calibration commands
    Cal {
        #[command(subcommand)]
        cmd: commands::calibrate::CalibrateCommand,
    },

    /// System commands
    Sys {
        #[command(subcommand)]
        cmd: commands::system::SystemCommand,
    },

    /// Show the current arming status
    Arming {
        #[command(subcommand)]
        cmd: commands::arming::ArmingCommand,
    },

    /// Trigger onboard motor test profiles
    Motor {
        #[command(subcommand)]
        cmd: commands::motor::MotorCommand,
    },

    /// Interact with the MAVLink servers
    #[cfg(feature = "mavlink")]
    Mavlink {
        #[command(subcommand)]
        cmd: commands::mavlink::MavlinkCommand,
    },

    /// View and modify system parameters
    Param {
        #[command(subcommand)]
        cmd: commands::params::ParamCommand,
    },

    /// Inspect various signals / sensors in the system
    Inspect {
        #[command(subcommand)]
        cmd: commands::inspect::InspectCommand,
    },
}
