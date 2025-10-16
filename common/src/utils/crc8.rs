/// Create a new look-up table for the CRC8 algorithm.
pub(crate) const fn new_crc8_lut() -> [u8; 256] {
    let mut crc_table = [0u8; 256];

    let mut i = 0;
    while i < 256 {
        let mut crc = i as u8;
        let mut j = 0;
        while j < 8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0xd5;
            } else {
                crc <<= 1;
            }
            j += 1;
        }
        crc_table[i] = crc;
        i += 1;
    }

    crc_table
}

const CRC8_LUT: [u8; 256] = new_crc8_lut();

/// Software based CRC8 implementation.
pub(crate) struct Crc8 {
    digest: u8,
}

impl Crc8 {
    pub const fn new() -> Self {
        Crc8 { digest: 0 }
    }

    pub fn compute(&mut self, data: &[u8]) {
        for e in data {
            self.digest = CRC8_LUT[(self.digest ^ e) as usize];
        }
    }

    pub fn reset(&mut self) {
        self.digest = 0;
    }

    pub fn get_checksum(&self) -> u8 {
        self.digest
    }
}
