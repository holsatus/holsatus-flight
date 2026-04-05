//! MTF-01 optical flow + lidar parser (MSP v2, 115200 baud).
//!
//! The MTF-01 pushes MSP v2 frames at ~100 Hz:
//!   - 0x1F02  OPTICAL_FLOW: quality(u8) + motion_x(i32 LE) + motion_y(i32 LE)
//!   - 0x1F01  RANGEFINDER:  quality(u8) + distance_mm(i32 LE)
//!
//! MSP v2 frame layout (bytes after the $X preamble):
//!   dir(1)  flags(1)  fn(2 LE)  size(2 LE)  payload[size]  crc8
//!
//! CRC8/DVB-S2 covers: flags + fn + size + payload (NOT $X or dir).

pub const OPTICAL_FLOW_FN: u16 = 0x1F02;
pub const RANGEFINDER_FN:  u16 = 0x1F01;

/// Maximum payload size accepted (optical flow = 9 bytes).
pub const MAX_PAYLOAD: usize = 16;

pub struct FlowSample {
    pub quality:  u8,
    pub motion_x: i32,
    pub motion_y: i32,
}

pub struct LidarSample {
    pub quality:     u8,
    pub distance_mm: i32,
}

pub enum Frame {
    Flow(FlowSample),
    Lidar(LidarSample),
}

/// Parse a complete frame from the 6 header bytes (after $X) and
/// the payload+crc slice.
///
/// Returns `None` on CRC mismatch, unknown function, or short payload.
/// `payload_plus_crc` must be at least `size + 1` bytes long, where
/// `size` is encoded in `hdr[4..6]`.
pub fn parse(hdr: [u8; 6], payload_plus_crc: &[u8]) -> Option<Frame> {
    let function = u16::from_le_bytes([hdr[2], hdr[3]]);
    let size     = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;

    if size > MAX_PAYLOAD || payload_plus_crc.len() < size + 1 {
        return None;
    }

    // CRC covers flags + fn + size + payload = hdr[1..] then payload bytes.
    let mut crc = 0u8;
    for &b in &hdr[1..] { crc = crc8_step(crc, b); }
    for &b in &payload_plus_crc[..size] { crc = crc8_step(crc, b); }
    if crc != payload_plus_crc[size] {
        return None;
    }

    let p = &payload_plus_crc[..size];
    match function {
        OPTICAL_FLOW_FN if p.len() >= 9 => Some(Frame::Flow(FlowSample {
            quality:  p[0],
            motion_x: i32::from_le_bytes([p[1], p[2], p[3], p[4]]),
            motion_y: i32::from_le_bytes([p[5], p[6], p[7], p[8]]),
        })),
        RANGEFINDER_FN if p.len() >= 5 => Some(Frame::Lidar(LidarSample {
            quality:     p[0],
            distance_mm: i32::from_le_bytes([p[1], p[2], p[3], p[4]]),
        })),
        _ => None,
    }
}

fn crc8_step(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 { (crc << 1) ^ 0xD5 } else { crc << 1 };
    }
    crc
}
