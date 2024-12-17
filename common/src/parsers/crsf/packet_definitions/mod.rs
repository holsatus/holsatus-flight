use crate::utils::crc8::Crc8;

use super::{
    packet_address::PacketAddress, packet_containers::RawPacket, packet_type::PacketType, Error,
    CRSF_MAX_LEN, CRSF_SYNC_BYTE,
};

pub mod device_ping;
pub mod link_statistics;
pub mod rc_channels_packed;

/// A trait encapsulationg a CRSF payload. This trait is used to encode and decode payloads
/// to and from byte slices, as well as convert into a [`RawPacket`]s for transmitting elsewhere.
#[allow(clippy::len_without_is_empty)]
pub trait AnyPayload
where
    Self: Sized,
{
    /// The length in bytes of this payload when serialized.
    const LEN: usize;

    /// Get the length in bytes of this payload when serialized.
    fn len(&self) -> usize {
        Self::LEN
    }

    /// Get the packet type of this payload.
    fn packet_type(&self) -> PacketType;

    /// Decode a payload from a slice. This must not include the `sync`, `len`, `type`, or `crc` bytes.
    fn decode(buf: &[u8]) -> Result<Self, Error>;

    /// Encode a payload into a mutable slice. This does not include the `sync`, `len`, `type`, or `crc` bytes.
    fn encode<'a>(&self, buf: &'a mut [u8]) -> Result<&'a [u8], Error>;
}

pub trait Payload: AnyPayload {
    /// Construct a new `RawPacket` from a `Packet`. This adds the `sync`, `len`, `type` bytes,
    /// and calculates and adds the `crc` byte. This constructor assumes the given packet is valid.
    fn to_raw_packet(&self) -> Result<RawPacket, Error> {
        self.to_raw_packet_with_sync(CRSF_SYNC_BYTE)
    }

    /// Construct a new `RawPacket` from a `Packet`. This adds the given `sync` byte, `len`, `type` bytes,
    /// and calculates and adds the `crc` byte. This constructor assumes the given packet is valid.
    /// Note that changing the sync byte is not officially supported by the CRSF protocol, but is used
    /// in some implementations as an "address" byte.
    fn to_raw_packet_with_sync(&self, sync_byte: u8) -> Result<RawPacket, Error> {
        let mut raw = RawPacket {
            buf: [0u8; CRSF_MAX_LEN],
            len: 4 + Self::LEN,
        };

        // Insert the payload into the packet
        if let Some(payload_buffer) = raw.buf.get_mut(3..) {
            self.encode(payload_buffer)?;
        } else {
            debug_assert!(false, "Failed to get payload buffer")
        }

        // Doing this after the encode ensures we do not change
        // the contents of the RawPacket if the payload encoding fails.
        raw.buf[0] = sync_byte;
        raw.buf[1] = 2 + Self::LEN as u8;
        raw.buf[2] = self.packet_type() as u8;

        // Calculate the CRC checksum
        let mut crc = Crc8::new();
        if let Some(crc_bytes) = raw.buf.get(2..3 + Self::LEN) {
            crc.compute(crc_bytes);
        } else {
            debug_assert!(false, "Failed to get crc bytes")
        }

        // Insert the calculated CRC into the packet
        if let Some(crc_byte) = raw.buf.get_mut(3 + Self::LEN) {
            *crc_byte = crc.get_checksum();
        } else {
            debug_assert!(false, "Failed to get crc byte")
        }

        raw.len = 4 + Self::LEN;

        Ok(raw)
    }
}

pub trait ExtendedPayload: AnyPayload {
    /// Construct a new `RawPacket` from a `Packet`. This adds the `sync`, `len`, `type`, `dst`, `src`
    /// bytes, and calculates and adds the `crc` byte. This constructor assumes the given packet is valid.
    fn to_raw_packet(&self, dst: PacketAddress, src: PacketAddress) -> Result<RawPacket, Error> {
        self.to_raw_packet_with_sync(CRSF_SYNC_BYTE, dst, src)
    }

    /// Construct a new `RawPacket` from a `Packet`. This adds the given `sync`, `len`, `type`, `dst`, `src`
    /// bytes, and calculates and adds the `crc` byte. This constructor assumes the given packet is valid.
    /// Note that changing the sync byte is not officially supported by the CRSF protocol, but is used
    /// in some implementations as an "address" byte.
    fn to_raw_packet_with_sync(
        &self,
        sync_byte: u8,
        dst: PacketAddress,
        src: PacketAddress,
    ) -> Result<RawPacket, Error> {
        let mut raw = RawPacket {
            buf: [0u8; CRSF_MAX_LEN],
            len: 6 + Self::LEN,
        };

        // Insert the payload into the packet
        if let Some(payload_buffer) = raw.buf.get_mut(5..) {
            self.encode(payload_buffer)?;
        } else {
            debug_assert!(false, "Failed to get payload buffer")
        }

        // Doing this after the encode ensures we do not change
        // the contents of the RawPacket if the payload encoding fails.
        raw.buf[0] = sync_byte;
        raw.buf[1] = 4 + Self::LEN as u8;
        raw.buf[2] = self.packet_type() as u8;
        raw.buf[3] = dst as u8;
        raw.buf[4] = src as u8;

        // Calculate the CRC checksum
        let mut crc = Crc8::new();
        if let Some(crc_bytes) = raw.buf.get(2..5 + Self::LEN) {
            crc.compute(crc_bytes);
        } else {
            debug_assert!(false, "Failed to get crc bytes")
        }

        // Insert the calculated CRC into the packet
        if let Some(crc_byte) = raw.buf.get_mut(5 + Self::LEN) {
            *crc_byte = crc.get_checksum();
        } else {
            debug_assert!(false, "Failed to get crc byte")
        }

        raw.len = 6 + Self::LEN;

        Ok(raw)
    }
}

macro_rules! impl_any_payload {
    ($module:ident, $name:ident) => {
        impl AnyPayload for $module::$name {
            const LEN: usize = $module::LEN;

            fn packet_type(&self) -> super::packet_type::PacketType {
                super::packet_type::PacketType::$name
            }

            fn decode(buf: &[u8]) -> Result<Self, super::Error> {
                let data: &[u8; $module::LEN] =
                    crate::utils::func::ref_array_start(buf).ok_or(super::Error::BufferError)?;
                Ok($module::raw_decode(data))
            }

            fn encode<'a>(&self, buf: &'a mut [u8]) -> Result<&'a [u8], super::Error> {
                let data: &mut [u8; $module::LEN] =
                    crate::utils::func::mut_array_start(buf).ok_or(super::Error::BufferError)?;
                $module::raw_encode(self, data);
                Ok(data)
            }
        }
    };
}

macro_rules! impl_payload {
    ($module:ident, $name:ident) => {
        impl_any_payload!($module, $name);
        impl Payload for $module::$name {}
    };
}

macro_rules! impl_extended_payload {
    ($module:ident, $name:ident) => {
        impl_any_payload!($module, $name);
        impl ExtendedPayload for $module::$name {}
    };
}

impl_payload!(link_statistics, LinkStatistics);
impl_payload!(rc_channels_packed, RcChannelsPacked);
impl_extended_payload!(device_ping, DevicePing);
