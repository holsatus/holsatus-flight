//! LinkStatistics packet and related functions/implementations

/// LinkStatistics payload length
pub const LEN: usize = 10;

/// Represents a LinkStatistics packet
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct LinkStatistics {
    pub uplink_rssi_1: u8,
    pub uplink_rssi_2: u8,
    pub uplink_link_quality: u8,
    pub uplink_snr: i8,
    pub active_antenna: u8,
    pub rf_mode: u8,
    pub uplink_tx_power: u8,
    pub downlink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_snr: i8,
}

/// The raw decoder (parser) for the LinkStatistics packet.
pub fn raw_decode(data: &[u8; LEN]) -> LinkStatistics {
    LinkStatistics {
        uplink_rssi_1: data[0],
        uplink_rssi_2: data[1],
        uplink_link_quality: data[2],
        uplink_snr: data[3] as i8,
        active_antenna: data[4],
        rf_mode: data[5],
        uplink_tx_power: data[6],
        downlink_rssi: data[7],
        downlink_link_quality: data[8],
        downlink_snr: data[9] as i8,
    }
}

/// The raw encoder (serializer) for the LinkStatistics packet.
pub fn raw_encode(link_statistics: &LinkStatistics, data: &mut [u8; LEN]) {
    data[0] = link_statistics.uplink_rssi_1;
    data[1] = link_statistics.uplink_rssi_2;
    data[2] = link_statistics.uplink_link_quality;
    data[3] = link_statistics.uplink_snr as u8;
    data[4] = link_statistics.active_antenna;
    data[5] = link_statistics.rf_mode;
    data[6] = link_statistics.uplink_tx_power;
    data[7] = link_statistics.downlink_rssi;
    data[8] = link_statistics.downlink_link_quality;
    data[9] = link_statistics.downlink_snr as u8;
}
