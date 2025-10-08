pub mod packet_address;
pub mod packet_containers;
pub mod packet_definitions;
pub mod packet_type;

use super::RcParser;
use crate::errors::{adapter::embedded_io::EmbeddedIoError, ParseError};

impl RcParser for CrsfParser {
    fn parse<'b>(
        &mut self,
        bytes: &'b [u8],
    ) -> (Option<Result<super::RcPacket, ParseError>>, &'b [u8]) {
        let (result, bytes) = self.push_bytes(bytes);

        match result {
            Some(Ok(raw_packet)) => match raw_packet.to_packet() {
                Ok(Packet::RcChannelsPacked(p)) => {
                    (Some(Ok(super::RcPacket::Channels(p.0))), bytes)
                }
                _ => {
                    error!("CrsfParser::parse: Unimplemented packet type");
                    (None, bytes)
                }
            },
            Some(Err(error)) => match error {
                Error::NoSyncByte => (Some(Err(ParseError::NoSyncronization)), bytes),
                Error::CrcMismatch { .. } => (Some(Err(ParseError::InvalidChecksum)), bytes),
                _ => (Some(Err(ParseError::InvalidData)), bytes),
            },
            None => (None, bytes),
        }
    }
}

const CRSF_MAX_LEN: usize = 64;
// Minimum data length, must include type and crc bytes
const MIN_LEN_BYTE: u8 = 2;
// Maximum data length, includes type, payload and crc bytes
const MAX_LEN_BYTE: u8 = CRSF_MAX_LEN as u8 - MIN_LEN_BYTE;

pub const CRSF_SYNC_BYTE: u8 = 0xC8;
const CRSF_HEADER_LEN: usize = 2;

use crate::utils::crc8::Crc8;
use embedded_io::Error as _;
use embedded_io_async::Read;
use packet_containers::{Packet, RawPacket};

use packet_type::PacketType;

/// Represents packet parsing errors
#[non_exhaustive]
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(thiserror::Error)]
pub enum Error {
    #[error("Io error: {0}")]
    IoError(#[from] EmbeddedIoError),
    #[error("No sync byte")]
    NoSyncByte,
    #[error("Unknown type index: {typ}")]
    InvalidType { typ: u8 },
    #[error("Type {typ:?} not implemented")]
    UnimplementedType { typ: PacketType },
    #[error("Packet {typ:?} not of extended format")]
    PacketNotExtended { typ: PacketType },
    #[error("Length of {len} is invalid")]
    InvalidLength { len: u8 },
    #[error("Address {addr} is invalid")]
    InvalidAddress { addr: u8 },
    #[error("Payload could not be parsed")]
    InvalidPayload,
    #[error("Crc mismatch, {exp} != {act}")]
    CrcMismatch { exp: u8, act: u8 },
    #[error("Payload could not be parsed")]
    BufferError,
}

impl <E: embedded_io::Error> From<E> for Error {
    fn from(value: E) -> Self {
        Error::IoError(value.kind().into())
    }
}

/// Annoying wrapper struct to adapt ReadExactError
struct Ree<E>(embedded_io::ReadExactError<E>);

impl <E: embedded_io::Error> From<Ree<E>> for Error {
    fn from(value: Ree<E>) -> Self {
        match value.0 {
            embedded_io::ReadExactError::UnexpectedEof => EmbeddedIoError::UnexpectedEof.into(),
            embedded_io::ReadExactError::Other(inner) => inner.into(),
        }
    }
}

/// Represents a packet reader
pub struct CrsfParser {
    state: usize,
    digest: Crc8,
    raw: RawPacket,
    type_check: bool,
}

impl CrsfParser {
    /// Creates a new PacketReader struct
    pub const fn new() -> Self {
        Self {
            state: 0,
            digest: Crc8::new(),
            raw: RawPacket::empty(),
            type_check: true,
        }
    }

    /// Resets reader's state
    ///
    /// Useful in situations when timeout is triggered but a packet is not parsed
    pub fn reset(&mut self) {
        self.state = 0;
        self.digest.reset();
        self.raw.len = 0; // Soft-reset the buffer
    }

    pub async fn read_packet(&mut self, reader: impl super::BufReadExt + Read) -> Result<Packet, Error> {
        self.read(reader).await?.to_packet()
    }

    pub async fn read(&mut self, mut reader: impl super::BufReadExt + Read) -> Result<&RawPacket, Error> {

        // Read until we reach a sync byte
        let skipped = reader.skip_until(CRSF_SYNC_BYTE).await.map_err(Ree)?;

        if skipped > 0 {
            warn!("Skipped {} bytes to find CRSF header", skipped);
        }

        let exp_len;
        match reader.fill_buf().await? {

            // Proper implementations should not reach here
            &[] => return Err(Error::IoError(EmbeddedIoError::UnexpectedEof)),

            // Guard against incorrect length byte
            &[len, ..] if len > CRSF_MAX_LEN as u8 || len < CRSF_HEADER_LEN as u8 => {
                // Dont consume if it might be a sync byte
                if len != CRSF_SYNC_BYTE {
                    reader.consume(1);
                }
                return Err(Error::InvalidLength { len: len as u8 })
            }

            // Received *some* amont of payload
            [len, payload @ ..] => {

                self.raw.buf[1] = *len;
                
                let rcv_len = payload.len();
                exp_len = self.raw.buf[1] as usize;
                
                // We received all bytes needed
                if rcv_len >= exp_len {
                    self.raw.buf[2..exp_len + 2]
                        .copy_from_slice(&payload[..exp_len]);
                    reader.consume(exp_len + 1);

                } else {
                    // Copy and consume received bytes
                    self.raw.buf[2..rcv_len + 2]
                        .copy_from_slice(payload);
                    reader.consume(rcv_len + 1);
                    
                    // Determine the region of the raw buffer we need to fill
                    let remaining = rcv_len - exp_len;
                    let range = (2 + rcv_len)..(2 + rcv_len + remaining);

                    reader.read_exact(&mut self.raw.buf[range]).await.map_err(Ree)?
                }
            }
        }

        // Compute and compare crc checksum
        let mut crc = Crc8::new();
        crc.compute(&self.raw.buf[2..exp_len + 1]);
        let act_crc = crc.get_checksum();
        let exp_crc = self.raw.buf[exp_len + 1];

        if act_crc != exp_crc {
            return Err(Error::CrcMismatch {
                exp: exp_crc,
                act: act_crc,
            });
        }

        // All good, finalize length
        self.raw.len = exp_len + CRSF_HEADER_LEN;
        return Ok(&self.raw);
    }

    /// Reads the first packet from the buffer
    pub fn push_bytes<'r, 'b>(
        &'r mut self,
        bytes: &'b [u8],
    ) -> (Option<Result<&'r RawPacket, Error>>, &'b [u8]) {
        let mut reader = crate::utils::bytes_reader::BytesReader::new(bytes);
        let packet = 'state_machine: loop {
            match self.state {
                0 => {
                    while let Some(sync_byte) = reader.next() {
                        if sync_byte == CRSF_SYNC_BYTE {
                            self.raw.buf[0] = sync_byte;
                            self.state = 1;
                            continue 'state_machine;
                        }
                    }

                    if reader.is_empty() {
                        break Some(Err(Error::NoSyncByte));
                    }
                }
                1 => {
                    let Some(len_byte) = reader.next() else {
                        break None;
                    };
                    match len_byte {
                        MIN_LEN_BYTE..=MAX_LEN_BYTE => {
                            self.raw.buf[1] = len_byte;
                            self.raw.len = CRSF_HEADER_LEN;
                            self.state = self.raw.len;
                        }
                        _ => {
                            self.reset();
                            break Some(Err(Error::InvalidLength { len: len_byte }));
                        }
                    }
                }
                n => {
                    if reader.is_empty() {
                        break None;
                    }

                    // Copy as many bytes as we need
                    let final_len = self.raw.buf[1] as usize + CRSF_HEADER_LEN;
                    let data = reader.next_n(final_len - self.raw.len);
                    self.raw.buf[self.raw.len..self.raw.len + data.len()].copy_from_slice(data);
                    self.raw.len += data.len();

                    // Validate that type is in PacketType enum
                    if let Some(type_byte) = self.raw.buf.get(2).copied() {
                        if self.type_check && PacketType::try_from(type_byte).is_err() {
                            self.reset();
                            break Some(Err(Error::InvalidType { typ: type_byte }));
                        }
                    }

                    // If we have received the CRC byte, do not use it in the digest
                    if self.raw.len == final_len {
                        self.digest.compute(&data[..data.len() - 1]);
                        let act_crc = self.digest.get_checksum();
                        let exp_crc = self.raw.buf[self.raw.len - 1];
                        if act_crc != exp_crc {
                            self.reset();
                            break Some(Err(Error::CrcMismatch {
                                exp: exp_crc,
                                act: act_crc,
                            }));
                        }
                    } else {
                        self.digest.compute(data);
                    }

                    if self.raw.len >= final_len {
                        self.digest.reset();
                        self.state = 0;
                        break Some(Ok(&self.raw));
                    }
                }
            }
        };

        (packet, reader.remaining())
    }

    /// Returns an interator over the given buffer. If the buffer contains
    /// packet of a valid format, the iterator will return `Ok(RawPacket)`. If
    /// the buffer contains invalid packets, the iterator will return
    /// `Err(Error)`. If the buffer is too small to parse, the iterator will
    /// yield. Once the iterator yields, all bytes in the buffer have been
    /// consumed.
    ///
    /// To get an iterator that returns `Packet`, use `iter_packets`.
    pub fn iter_raw_packets<'a, 'b>(&'a mut self, buf: &'b [u8]) -> IterRawPackets<'a, 'b> {
        IterRawPackets { parser: self, buf }
    }

    /// Returns an iterator over the given buffer. If the buffer contains
    /// packets of a valid format, the iterator will return `Ok(Packet)`. If the
    /// buffer contains invalid packets, the iterator will return `Err(Error)`.
    /// If the buffer is too small to parse, the iterator will yield. Once the
    /// iterator yields, all bytes in the buffer have been consumed.
    ///
    /// To get an iterator that returns `RawPacket`, use `iter_raw_packets`.
    pub fn iter_packets<'a, 'b>(&'a mut self, buf: &'b [u8]) -> IterPackets<'a, 'b> {
        IterPackets { parser: self, buf }
    }
}

/// An iterator over a buffer that yield `RawPacket` instances, or `Error` in
/// case of currupt data. This iterator will consume the and process the entire
/// buffer. For an iterator that also parses the packets into `Packet`
/// instances, use `IterPackets` instead.
pub struct IterRawPackets<'a, 'b> {
    parser: &'a mut CrsfParser,
    buf: &'b [u8],
}

impl<'a, 'b> Iterator for IterRawPackets<'a, 'b> {
    type Item = Result<RawPacket, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.buf.is_empty() {
            return None;
        }
        let result;
        (result, self.buf) = self.parser.push_bytes(self.buf);
        result.map(|raw| raw.cloned())
    }
}

/// An iterator over a buffer that return parsed `Packet` instances, or `Error`
/// in case of currupt data. This iterator will consume the and process the
/// entire buffer.
pub struct IterPackets<'a, 'b> {
    parser: &'a mut CrsfParser,
    buf: &'b [u8],
}

impl<'a, 'b> Iterator for IterPackets<'a, 'b> {
    type Item = Result<Packet, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.buf.is_empty() {
            return None;
        }
        let result;
        (result, self.buf) = self.parser.push_bytes(self.buf);
        result.map(|res| match res {
            Ok(raw) => raw.to_packet(),
            Err(err) => Err(err),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{
        packet_definitions::rc_channels_packed::RcChannelsPacked, CrsfParser, Error, Packet,
        PacketType, CRSF_SYNC_BYTE,
    };

    #[test]
    fn test_packet_reader_waiting_for_sync_byte() {
        let mut reader = CrsfParser::new();

        let typ = PacketType::RcChannelsPacked as u8;

        for _ in 0..2 {
            // Garbage
            assert!(matches!(
                reader.push_bytes(&[1, 2, 3]).0,
                Some(Err(Error::NoSyncByte))
            ));
            // More garbage
            assert!(matches!(
                reader.push_bytes(&[254, 255]).0,
                Some(Err(Error::NoSyncByte))
            ));
            // Sync
            assert!(reader.push_bytes(&[CRSF_SYNC_BYTE]).0.is_none());
            // Len
            assert!(reader.push_bytes(&[24]).0.is_none());
            // Type
            assert!(reader.push_bytes(&[typ]).0.is_none());
            // Payload
            assert!(reader.push_bytes(&[0; 22]).0.is_none());

            // Checksum
            let result = reader.push_bytes(&[239]).0.expect("result expected");

            let raw_packet = result.expect("raw packet expected");
            let packet = raw_packet.to_packet().expect("packet expected");

            match packet {
                Packet::RcChannelsPacked(ch) => {
                    ch.0.iter().all(|&x| x == 0);
                }
                _ => panic!("unexpected packet type"),
            }
        }
    }

    #[test]
    fn test_parse_next_packet() {
        let mut reader = CrsfParser::new();

        let typ = PacketType::RcChannelsPacked;

        // Sync
        assert!(reader.push_bytes(&[CRSF_SYNC_BYTE]).0.is_none());
        // Len
        assert!(reader.push_bytes(&[24]).0.is_none());
        // Type
        assert!(reader.push_bytes(&[typ as u8]).0.is_none());
        // Payload
        assert!(reader.push_bytes(&[0; 22]).0.is_none());
        // Checksum
        let result = reader.push_bytes(&[239]).0.expect("result expected");

        let raw_packet = result.expect("raw packet expected");
        let packet = raw_packet.to_packet().expect("packet expected");

        match packet {
            Packet::RcChannelsPacked(ch) => {
                ch.0.iter().all(|&x| x == 0);
            }
            _ => panic!("unexpected packet type"),
        }
    }


    #[test]
    fn test_async_read_segments() {
        // similar to the doc-test at the top
        let mut reader = CrsfParser::new();
        let data: &[u8] = &[0xc8, 24, 0x16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 239];
        
        let mut buffer = data.as_ref();
        
        futures_executor::block_on(async {
            let result = reader.read(&mut buffer).await;
            assert_eq!(result.unwrap().to_packet().unwrap(), Packet::RcChannelsPacked(RcChannelsPacked([0; 16])));

            assert_eq!(buffer.len(), 0);
        })
    }


    #[test]
    fn test_async_read_segments_double() {
        // similar to the doc-test at the top
        let mut reader = CrsfParser::new();
        let data: &[u8] = &[
            0xc8, 24, 0x16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 239,
            0xc8, 24, 0x16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 239,
        ];

        let mut buffer = data.as_ref();
        
        futures_executor::block_on(async {
            let result = reader.read(&mut buffer).await;
            assert_eq!(result.unwrap().to_packet().unwrap(), Packet::RcChannelsPacked(RcChannelsPacked([0; 16])));

            assert_eq!(buffer.len(), 26);

            let result = reader.read(&mut buffer).await;
            assert_eq!(result.unwrap().to_packet().unwrap(), Packet::RcChannelsPacked(RcChannelsPacked([0; 16])));
          
            assert_eq!(buffer.len(), 0);
        })
    }

    #[test]
    fn test_push_segments() {
        // similar to the doc-test at the top
        let mut reader = CrsfParser::new();
        let data: &[&[u8]] = &[&[0xc8, 24, 0x16], &[0; 22], &[239]];
        for (_, input_buf) in data.iter().enumerate() {
            for (_, result) in reader.iter_packets(input_buf).enumerate() {
                match result {
                    Ok(Packet::RcChannelsPacked(rc_channels)) => {
                        assert_eq!(rc_channels, RcChannelsPacked([0u16; 16]))
                    }
                    _ => panic!("This data should parse succesfully"),
                }
            }
        }
    }

    #[test]
    fn test_parse_full_packet() {
        let mut reader = CrsfParser::new();

        let typ = PacketType::RcChannelsPacked;

        #[rustfmt::skip]
        let data = [
            // Sync
            CRSF_SYNC_BYTE,
            // Len
            24,
            // Type
            typ as u8,
            // Payload
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // Checksum
            239,
        ];

        let result = reader
            .push_bytes(data.as_slice())
            .0
            .expect("result expected");

        let raw_packet = result.expect("raw packet expected");
        let packet = raw_packet.to_packet().expect("packet expected");

        match packet {
            Packet::RcChannelsPacked(ch) => {
                ch.0.iter().all(|&x| x == 0);
            }
            _ => panic!("unexpected packet type"),
        }
    }

    #[test]
    fn test_parse_next_packet_with_validation_error() {
        let mut reader = CrsfParser::new();

        // Sync
        assert!(reader.push_bytes(&[CRSF_SYNC_BYTE]).0.is_none());
        // Len
        assert!(reader.push_bytes(&[24]).0.is_none());
        // Type
        assert!(reader
            .push_bytes(&[PacketType::RcChannelsPacked as u8])
            .0
            .is_none());
        // Payload
        assert!(reader.push_bytes(&[0; 22]).0.is_none());
        // Checksum
        let result = reader.push_bytes(&[42]).0.expect("result expected");

        assert!(matches!(
            result,
            Err(Error::CrcMismatch { act: 239, exp: 42 })
        ));
    }
}

struct BufReader<R: Read, const N: usize> {
    reader: R,
    buf: [u8; N],
    len: usize,
}

impl <R: Read, const N: usize> BufReader<R, N> {
    pub const fn new(reader: R) -> Self {
        BufReader {
            reader,
            buf: [0; N],
            len: 0,
        }
    }
}

impl <R: Read, const N: usize> embedded_io_async::ErrorType for BufReader<R, N> {
    type Error = R::Error;
}

impl <R: Read, const N: usize> embedded_io_async::BufRead for BufReader<R, N> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        let bytes = self.reader.read(&mut self.buf[self.len..]).await?;
        Ok(&self.buf[self.len..bytes + self.len])
    }

    fn consume(&mut self, amt: usize) {
        self.len = (self.len + amt).min(self.buf.len())
    }
}

impl <R: Read, const N: usize> embedded_io_async::Read for BufReader<R, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if self.len > 0 {
            let amt = buf.len().min(self.len);
            buf[..amt].copy_from_slice(&self.buf[..amt]);
            self.len -= amt;
            Ok(amt)
        } else {
            self.reader.read(buf).await
        }
    }
}
