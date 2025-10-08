use crate::utils::bytes_reader::BytesReader;

use super::{RcPacket, RcParser};
use crate::errors::ParseError;

impl RcParser for SbusParser {
    fn parse<'b>(&mut self, bytes: &'b [u8]) -> (Option<Result<RcPacket, ParseError>>, &'b [u8]) {
        let (result, buf) = self.push_bytes(bytes);
        (
            result.map(|opt| opt.map(|sbus| RcPacket::Channels(sbus.channels))),
            buf,
        )
    }
}

// Important bytes for correctnes checks
const FLAG_MASK: u8 = 0b11110000;
const HEAD_BYTE: u8 = 0b00001111;
const FOOT_BYTE: u8 = 0b00000000;

// Number of bytes in SBUS message
const PACKET_SIZE: usize = 25;

#[derive(Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SbusPacket {
    pub channels: [u16; 16],
    pub d1: bool,
    pub d2: bool,
    pub failsafe: bool,
    pub frame_lost: bool,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum State {
    AwaitingHead,
    Reading(usize),
}

#[derive(Debug)]
pub struct SbusParser {
    buffer: [u8; PACKET_SIZE],
    state: State,
    len: usize,
}

impl SbusParser {
    pub fn new() -> SbusParser {
        SbusParser {
            buffer: [0; PACKET_SIZE],
            state: State::AwaitingHead,
            len: 0,
        }
    }

    /// Push single `u8` byte into buffer.
    pub fn push_byte(&mut self, byte: u8) -> Option<Result<SbusPacket, ParseError>> {
        match self.state {
            State::AwaitingHead => {
                if byte == HEAD_BYTE {
                    self.buffer[0] = byte;
                    self.state = State::Reading(1);
                }
            }
            State::Reading(n) if n == PACKET_SIZE - 1 => {
                self.buffer[n] = byte;
                self.state = State::Reading(n + 1);
                return Some(self.try_parse());
            }
            State::Reading(n) => {
                self.buffer[n] = byte;
                self.state = State::Reading(n + 1);
            }
        }

        None
    }

    pub async fn read(&mut self, mut reader: impl super::BufReadExt + embedded_io_async::Read) -> Result<SbusPacket, ParseError> {

        // Read until we reach a sync byte
        let skipped = reader.skip_until(HEAD_BYTE).await.map_err(|_|ParseError::NoSyncronization)?;
        self.buffer[0] = HEAD_BYTE;

        if skipped > 0 { warn!("Skipped {} bytes to find SBUS header", skipped) }

        reader.read_exact(&mut self.buffer[1..]).await.map_err(|_|ParseError::NoSyncronization)?;

        self.try_parse()
    }

    /// Push array of `u8` bytes into buffer.
    pub fn push_bytes<'b>(
        &mut self,
        bytes: &'b [u8],
    ) -> (Option<Result<SbusPacket, ParseError>>, &'b [u8]) {
        let mut reader = BytesReader::new(bytes);

        let packet = 'state_machine: loop {
            match self.state {
                State::AwaitingHead => {
                    while let Some(byte) = reader.next() {
                        if byte == HEAD_BYTE {
                            self.buffer[0] = byte;
                            self.state = State::Reading(1);
                            continue 'state_machine;
                        }
                    }

                    if reader.is_empty() {
                        break Some(Err(ParseError::NoSyncronization));
                    }
                }
                State::Reading(ref mut n) => {
                    if reader.is_empty() {
                        break None;
                    }

                    // Copy bytes from reader to buffer, as many as possible
                    let bytes = reader.next_n(PACKET_SIZE - *n);
                    self.buffer[*n..*n + bytes.len()].copy_from_slice(bytes);
                    *n += bytes.len();

                    // If we have filled the buffer, try to parse it
                    if *n == PACKET_SIZE {
                        let parsed = match self.try_parse() {
                            Ok(parsed) => parsed,
                            Err(e) => {
                                // If the buffer contains another head byte, left-shift everything
                                // until that is first, and set State::Data(n) correspondingly
                                if let Some(pos) =
                                    self.buffer.iter().skip(1).position(|&b| b == HEAD_BYTE)
                                {
                                    self.buffer.copy_within(pos + 1.., 0);

                                    // Calculate index to the next byte to read
                                    let mut index = 0;
                                    if let State::Reading(p) = self.state {
                                        index = p - pos - 1;
                                    }

                                    self.state = State::Reading(index);
                                } else {
                                    self.state = State::AwaitingHead;
                                }

                                break Some(Err(e));
                            }
                        };

                        break Some(Ok(parsed));
                    }
                }
            }
        };

        (packet, reader.remaining())
    }

    /// Attempts to parse a valid SBUS packet from the buffer
    pub fn try_parse(&self) -> Result<SbusPacket, ParseError> {
        // Check if entire frame is valid
        if self.state != State::Reading(PACKET_SIZE) || !self.valid_frame() {
            return Err(ParseError::InvalidData);
        }

        // Short-hand reference to buffer
        let buf = &self.buffer;

        // Initialize channels with 11-bit mask
        let mut ch: [u16; 16] = [0x07FF; 16];

        // Trust me bro
        ch[0] &= (buf[1] as u16) | (buf[2] as u16) << 8;
        ch[1] &= (buf[2] as u16) >> 3 | (buf[3] as u16) << 5;
        ch[2] &= (buf[3] as u16) >> 6 | (buf[4] as u16) << 2 | (buf[5] as u16) << 10;
        ch[3] &= (buf[5] as u16) >> 1 | (buf[6] as u16) << 7;
        ch[4] &= (buf[6] as u16) >> 4 | (buf[7] as u16) << 4;
        ch[5] &= (buf[7] as u16) >> 7 | (buf[8] as u16) << 1 | (buf[9] as u16) << 9;
        ch[6] &= (buf[9] as u16) >> 2 | (buf[10] as u16) << 6;
        ch[7] &= (buf[10] as u16) >> 5 | (buf[11] as u16) << 3;

        ch[8] &= (buf[12] as u16) | (buf[13] as u16) << 8;
        ch[9] &= (buf[13] as u16) >> 3 | (buf[14] as u16) << 5;
        ch[10] &= (buf[14] as u16) >> 6 | (buf[15] as u16) << 2 | (buf[16] as u16) << 10;
        ch[11] &= (buf[16] as u16) >> 1 | (buf[17] as u16) << 7;
        ch[12] &= (buf[17] as u16) >> 4 | (buf[18] as u16) << 4;
        ch[13] &= (buf[18] as u16) >> 7 | (buf[19] as u16) << 1 | (buf[20] as u16) << 9;
        ch[14] &= (buf[20] as u16) >> 2 | (buf[21] as u16) << 6;
        ch[15] &= (buf[21] as u16) >> 5 | (buf[22] as u16) << 3;

        let flag_byte = buf[23];

        Ok(SbusPacket {
            channels: ch,
            d1: is_flag_set(flag_byte, 0),
            d2: is_flag_set(flag_byte, 1),
            frame_lost: is_flag_set(flag_byte, 2),
            failsafe: is_flag_set(flag_byte, 3),
        })
    }

    /// Returns `true` if the buffer contains a valid SBUS frame
    fn valid_frame(&self) -> bool {
        // compare buffer contents with expected values
        self.buffer[0] == HEAD_BYTE
            && self.buffer[PACKET_SIZE - 1] == FOOT_BYTE
            && self.buffer[PACKET_SIZE - 2] & FLAG_MASK == 0
    }
}

#[inline(always)]
fn is_flag_set(flag_byte: u8, shift_by: u8) -> bool {
    (flag_byte >> shift_by) & 1 == 1
}

#[cfg(test)]
mod tests {
    use hex_literal::hex;

    use crate::errors::ParseError;
    use crate::parsers::sbus;

    const RAW_BYTES: [u8; 25] =
        hex!("0F E0 03 1F 58 C0 07 16 B0 80 05 2C 60 01 0B F8 C0 07 00 00 00 00 00 03 00");

    const DECODED_PACKET: sbus::SbusPacket = sbus::SbusPacket {
        channels: [
            992, 992, 352, 992, 352, 352, 352, 352, 352, 352, 992, 992, 0, 0, 0, 0,
        ],
        d1: true,
        d2: true,
        failsafe: false,
        frame_lost: false,
    };

    #[test]
    fn test_sbus_parse_basic() {
        let mut parser = sbus::SbusParser::new();

        // Push full message
        _ = parser.push_bytes(&RAW_BYTES);

        assert_eq!(Ok(DECODED_PACKET), parser.try_parse());
    }

    #[test]
    fn test_sbus_parse_with_shifting() {
        let mut parser = sbus::SbusParser::new();

        // Push message within other garbage bytes
        let _ = parser.push_bytes(&RAW_BYTES[5..15]);
        let (result, _) = parser.push_bytes(&RAW_BYTES); // Actual message
        let _ = parser.push_bytes(&RAW_BYTES[5..15]);

        assert_eq!(Some(Ok(DECODED_PACKET)), result);
    }

    #[test]
    fn test_sbus_parse_mid_parse_reset() {
        let mut parser = sbus::SbusParser::new();

        let (result, _) = parser.push_bytes(&RAW_BYTES[..10]); // False start
        assert_eq!(None, result);

        let (result, _) = parser.push_bytes(&RAW_BYTES[..15]); // Start of actual message
        assert_eq!(Some(Err(ParseError::InvalidData)), result);

        let (result, _) = parser.push_bytes(&RAW_BYTES[15..]); // End of actual message
        assert_eq!(Some(Ok(DECODED_PACKET)), result);
    }

    #[test]
    fn test_sbus_error_missing_byte() {
        let mut parser = sbus::SbusParser::new();

        // Push bytes with one missing (and some extra bytes)
        _ = parser.push_bytes(&RAW_BYTES[0..7]);
        _ = parser.push_bytes(&RAW_BYTES[8..25]);
        _ = parser.push_bytes(&RAW_BYTES[8..10]);

        assert_eq!(Err(ParseError::InvalidData), parser.try_parse());
    }

    #[test]
    fn test_sbus_error_too_short() {
        let mut parser = sbus::SbusParser::new();

        // Push message that is too short
        _ = parser.push_bytes(&RAW_BYTES[0..23]);

        assert_eq!(Err(ParseError::InvalidData), parser.try_parse());
    }

    #[test]
    fn test_sbus_parse_two_parter() {
        let mut parser = sbus::SbusParser::new();

        // Push first bytes of message
        let (result, _) = parser.push_bytes(&RAW_BYTES[0..10]);

        assert_eq!(None, result);

        // Push remaining message
        let (result, _) = parser.push_bytes(&RAW_BYTES[10..25]);

        assert_eq!(Some(Ok(DECODED_PACKET)), result);
    }
}
