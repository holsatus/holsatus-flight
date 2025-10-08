use embedded_cli::Command;
use ufmt::uwrite;

use crate::shell::UBuffer;

#[derive(Command)]
#[command(help_title = "MAVLink commands")]
pub enum MavlinkCommand {
    /// View the status of the MAVLink servers
    Status,

    /// Set the IDs of the MAVLink servers
    Id {
        /// Set the system ID of the MAVLink server
        #[arg(short = 's', long)]
        set_sys: Option<u8>,

        /// Set the component ID of the MAVLink server
        #[arg(short = 'c', long)]
        set_comp: Option<u8>,
    },
}

impl super::CommandHandler for MavlinkCommand {
    async fn handler(
        &self,
        mut serial: impl embedded_io_async::Read<Error = embedded_io::ErrorKind>
            + embedded_io_async::Write<Error = embedded_io::ErrorKind>,
    ) -> Result<(), embedded_io::ErrorKind> {
        match self {
            MavlinkCommand::Status => {
                serial
                    .write_all(b"Fetching MAVLink status is currently not available\n\r")
                    .await?;
            }
            MavlinkCommand::Id { set_sys, set_comp } => {
                if let Some(set_sys) = set_sys {
                    let mut buffer = UBuffer::<32>::new();
                    uwrite!(buffer, "Setting system ID to {}\n\r", set_sys)?;
                    serial.write_all(&buffer.inner).await?;
                    crate::mavlink::MAV_REQUEST
                        .send(crate::mavlink::Request::SetSystemId { id: *set_sys })
                        .await;
                }
                if let Some(set_comp) = set_comp {
                    let mut buffer = UBuffer::<32>::new();
                    uwrite!(buffer, "Setting component ID to {}\n\r", set_comp)?;
                    serial.write_all(&buffer.inner).await?;
                    crate::mavlink::MAV_REQUEST
                        .send(crate::mavlink::Request::SetComponentId { id: *set_comp })
                        .await;
                }
            }
        }

        Ok(())
    }
}
