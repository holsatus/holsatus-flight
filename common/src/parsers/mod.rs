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
