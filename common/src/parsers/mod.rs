use embedded_io_async::Read;

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

// TODO: Standardize interface for parsers which operate directly on a reader.
// But maybe the `to_fill` and `consume` method, which is reader agnostic, would
// be more general purpose?

#[allow(async_fn_in_trait)]
pub trait BufParser {
    type Packet;
    type Error;
    async fn read_from<R: BufReadExt + Read>(&mut self, reader: R) -> Result<Self::Packet, BufParserError<Self::Error, R::Error>>;
}

pub enum BufParserError<P, R> {
    Parser(P),
    Reader(R),
    ReaderEof,
}