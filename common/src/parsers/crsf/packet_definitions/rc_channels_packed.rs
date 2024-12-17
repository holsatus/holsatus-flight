//! RcChannelsPacked packet and related functions/implementations

/// RcChannelsPacked payload length
pub const LEN: usize = 22;

/// Represents a RcChannelsPacked packet
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RcChannelsPacked(pub [u16; 16]);

/// The raw decoder (parser) for the RcChannelsPacked packet.
pub fn raw_decode(data: &[u8; LEN]) -> RcChannelsPacked {
    // Convert u8 to u16 to make room for bit shifting
    let data: [u16; LEN] = core::array::from_fn(|i| data[i] as u16);

    const MASK_11BIT: u16 = 0x07FF;
    let mut ch = [MASK_11BIT; 16];

    ch[0] &= data[0] | data[1] << 8;
    ch[1] &= data[1] >> 3 | data[2] << 5;
    ch[2] &= data[2] >> 6 | data[3] << 2 | data[4] << 10;
    ch[3] &= data[4] >> 1 | data[5] << 7;
    ch[4] &= data[5] >> 4 | data[6] << 4;
    ch[5] &= data[6] >> 7 | data[7] << 1 | data[8] << 9;
    ch[6] &= data[8] >> 2 | data[9] << 6;
    ch[7] &= data[9] >> 5 | data[10] << 3;
    ch[8] &= data[11] | data[12] << 8;
    ch[9] &= data[12] >> 3 | data[13] << 5;
    ch[10] &= data[13] >> 6 | data[14] << 2 | data[15] << 10;
    ch[11] &= data[15] >> 1 | data[16] << 7;
    ch[12] &= data[16] >> 4 | data[17] << 4;
    ch[13] &= data[17] >> 7 | data[18] << 1 | data[19] << 9;
    ch[14] &= data[19] >> 2 | data[20] << 6;
    ch[15] &= data[20] >> 5 | data[21] << 3;

    RcChannelsPacked(ch)
}

/// The raw encoder (serializer) for the RcChannelsPacked packet.
pub fn raw_encode(ch: &RcChannelsPacked, data: &mut [u8; LEN]) {
    let ch = &ch.0;

    data[0] = (ch[0]) as u8;
    data[1] = (ch[0] >> 8 | ch[1] << 3) as u8;
    data[2] = (ch[1] >> 5 | ch[2] << 6) as u8;
    data[3] = (ch[2] >> 2) as u8;
    data[4] = (ch[2] >> 10 | ch[3] << 1) as u8;
    data[5] = (ch[3] >> 7 | ch[4] << 4) as u8;
    data[6] = (ch[4] >> 4 | ch[5] << 7) as u8;
    data[7] = (ch[5] >> 1) as u8;
    data[8] = (ch[5] >> 9 | ch[6] << 2) as u8;
    data[9] = (ch[6] >> 6 | ch[7] << 5) as u8;
    data[10] = (ch[7] >> 3) as u8;
    data[11] = (ch[8]) as u8;
    data[12] = (ch[8] >> 8 | ch[9] << 3) as u8;
    data[13] = (ch[9] >> 5 | ch[10] << 6) as u8;
    data[14] = (ch[10] >> 2) as u8;
    data[15] = (ch[10] >> 10 | ch[11] << 1) as u8;
    data[16] = (ch[11] >> 7 | ch[12] << 4) as u8;
    data[17] = (ch[12] >> 4 | ch[13] << 7) as u8;
    data[18] = (ch[13] >> 1) as u8;
    data[19] = (ch[13] >> 9 | ch[14] << 2) as u8;
    data[20] = (ch[14] >> 6 | ch[15] << 5) as u8;
    data[21] = (ch[15] >> 3) as u8;
}
