use crate::errors::ParseError;

pub mod crsf;
pub mod sbus;

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RcPacket {
    Channels([u16; 16]),
    Telemetry,
}

pub trait RcParser {
    fn parse<'b>(&mut self, bytes: &'b [u8]) -> (Option<Result<RcPacket, ParseError>>, &'b [u8]);
}

#[allow(async_fn_in_trait)]
pub trait BufReadExt: embedded_io_async::BufRead {
    async fn skip_until(
        &mut self,
        delim: u8,
    ) -> Result<usize, embedded_io::ReadExactError<Self::Error>> {
        let mut read: usize = 0;
        loop {
            let (done, used) = {
                let available = self.fill_buf().await?;

                if available.is_empty() {
                    return Err(embedded_io::ReadExactError::UnexpectedEof);
                }

                match available.iter().position(|p| *p == delim) {
                    Some(i) => (true, i + 1),
                    None => (false, available.len()),
                }
            };
            self.consume(used);
            read += used;
            if done || used == 0 {
                return Ok(read);
            }
        }
    }
}

impl<R: embedded_io_async::BufRead> BufReadExt for R {}
