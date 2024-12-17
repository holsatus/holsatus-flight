//! DevicePing packet and related functions/implementations

/// DevicePing payload length
pub const LEN: usize = 0;

/// Represents a DevicePing packet
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DevicePing;

/// The raw decoder (parser) for the DevicePing packet.
pub fn raw_decode(_data: &[u8; LEN]) -> DevicePing {
    DevicePing
}

/// The raw encoder (serializer) for the DevicePing packet.
pub fn raw_encode(_device_ping: &DevicePing, _data: &mut [u8; LEN]) {}
