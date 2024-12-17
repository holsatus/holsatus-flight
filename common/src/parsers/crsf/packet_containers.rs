use super::{
    packet_address::PacketAddress,
    packet_definitions::{
        device_ping::DevicePing, link_statistics::LinkStatistics,
        rc_channels_packed::RcChannelsPacked, AnyPayload,
    },
    packet_type::PacketType,
    Error, CRSF_MAX_LEN,
};

/// Represents a packet
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Packet {
    LinkStatistics(LinkStatistics),
    RcChannelsPacked(RcChannelsPacked),
    Extended {
        src: PacketAddress,
        dst: PacketAddress,
        packet: ExtendedPacket,
    },
}

#[non_exhaustive]
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExtendedPacket {
    DevicePing(DevicePing),
}

/// Represents a raw packet (not parsed)
#[derive(Clone, Copy, Debug)]
pub struct RawPacket {
    pub(crate) buf: [u8; CRSF_MAX_LEN],
    pub(crate) len: usize,
}

impl RawPacket {
    pub(crate) const fn empty() -> RawPacket {
        RawPacket {
            buf: [0u8; CRSF_MAX_LEN],
            len: 0,
        }
    }

    /// Create a new RawPacket from the given slice. The slice must be
    /// at most `CRSF_MAX_LEN`bytes long.
    pub fn new(slice: &[u8]) -> Result<RawPacket, Error> {
        let mut packet = RawPacket {
            buf: [0u8; CRSF_MAX_LEN],
            len: slice.len(),
        };

        packet
            .buf
            .get_mut(..slice.len())
            .ok_or(Error::BufferError)?
            .copy_from_slice(slice);

        Ok(packet)
    }

    /// Get the slice of the raw packets buffer
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len.min(CRSF_MAX_LEN)]
    }

    /// Convert the raw packet into a parsed packet
    pub fn to_packet(&self) -> Result<Packet, Error> {
        if let [_, _, typ, payload @ .., _] = self.as_slice() {
            let typ = PacketType::try_from(*typ).map_err(|_| Error::InvalidType { typ: *typ })?;
            match typ {
                PacketType::RcChannelsPacked => {
                    RcChannelsPacked::decode(payload).map(Packet::RcChannelsPacked)
                }
                PacketType::LinkStatistics => {
                    LinkStatistics::decode(payload).map(Packet::LinkStatistics)
                }
                typ if typ.is_extended() => {
                    if let [dst, src, payload @ ..] = payload {
                        let dst = PacketAddress::try_from(*dst)
                            .map_err(|_| Error::InvalidAddress { addr: *dst })?;
                        let src = PacketAddress::try_from(*src)
                            .map_err(|_| Error::InvalidAddress { addr: *src })?;
                        match typ {
                            PacketType::DevicePing => {
                                DevicePing::decode(payload).map(ExtendedPacket::DevicePing)
                            }
                            _ => Err(Error::UnimplementedType { typ }),
                        }
                        .map(|packet| Packet::Extended {
                            src,
                            dst,
                            packet,
                        })
                    } else {
                        Err(Error::BufferError)
                    }
                }
                typ => Err(Error::UnimplementedType { typ }),
            }
        } else {
            Err(Error::BufferError)
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::parsers::crsf::packet_address::PacketAddress;
    use crate::parsers::crsf::packet_containers::{ExtendedPacket, Packet};
    use crate::parsers::crsf::packet_definitions::device_ping::DevicePing;
    use crate::parsers::crsf::packet_definitions::link_statistics::LinkStatistics;
    use crate::parsers::crsf::packet_definitions::rc_channels_packed::RcChannelsPacked;
    use crate::parsers::crsf::packet_definitions::{ExtendedPayload, Payload};
    use crate::parsers::crsf::CRSF_SYNC_BYTE;

    #[test]
    fn test_rc_channels_packed_dump_and_parse() {
        let orig = RcChannelsPacked([0x7FF; 16]);

        let raw = orig.to_raw_packet().unwrap();
        let mut expected_data: [u8; 26] = [0xff; 26];
        expected_data[0] = CRSF_SYNC_BYTE;
        expected_data[1] = 24;
        expected_data[2] = 0x16;
        expected_data[25] = 143;
        assert_eq!(raw.as_slice(), expected_data.as_slice());

        let parsed = raw.to_packet().unwrap();
        assert!(matches!(parsed, Packet::RcChannelsPacked(parsed) if parsed == orig));
    }

    #[test]
    fn test_link_statistics_dump_and_parse() {
        let orig = LinkStatistics {
            uplink_rssi_1: 16,
            uplink_rssi_2: 19,
            uplink_link_quality: 99,
            uplink_snr: -105,
            active_antenna: 1,
            rf_mode: 2,
            uplink_tx_power: 3,
            downlink_rssi: 8,
            downlink_link_quality: 88,
            downlink_snr: -108,
        };

        let raw = orig.to_raw_packet().unwrap();
        let expected_data = [
            CRSF_SYNC_BYTE,
            12,
            0x14,
            16,
            19,
            99,
            151,
            1,
            2,
            3,
            8,
            88,
            148,
            252,
        ];
        assert_eq!(raw.as_slice(), expected_data.as_slice());

        let parsed = raw.to_packet().unwrap();
        assert!(matches!(parsed, Packet::LinkStatistics(parsed) if parsed == orig));
    }

    #[test]
    fn test_device_ping_dump_and_parse() {
        let orig = DevicePing;

        let raw = orig
            .to_raw_packet(PacketAddress::Broadcast, PacketAddress::FlightController)
            .unwrap();
        // TODO: maybe check against an expected_data

        let parsed = raw.to_packet().unwrap();
        assert!(
            matches!(parsed, Packet::Extended { dst: PacketAddress::Broadcast, src: PacketAddress::FlightController, packet: ExtendedPacket::DevicePing(parsed) } if parsed == orig)
        );
    }
}
