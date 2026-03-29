//! Async DPS310 barometer driver over I2C.
//!
//! Generic over `embedded_hal_async::i2c::I2c`.
//! The register map is identical to the SPI variant; only transport differs.
//!
//! I2C address: 0x77 when SDO=HIGH (default), 0x76 when SDO=LOW.

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

// ── Register addresses ────────────────────────────────────────────────────────

const REG_PSR_B2:     u8 = 0x00;
const REG_TMP_B2:     u8 = 0x03;
const REG_PRS_CFG:    u8 = 0x06;
const REG_TMP_CFG:    u8 = 0x07;
const REG_MEAS_CFG:   u8 = 0x08;
const REG_CFG_REG:    u8 = 0x09;
const REG_RESET:      u8 = 0x0C;
const REG_PRODUCT_ID: u8 = 0x0D;
const REG_COEF:       u8 = 0x10;

const PRODUCT_ID: u8 = 0x10;

/// I2C address when SDO pin is pulled high (default on most FCs).
pub const ADDR_SDO_HIGH: u8 = 0x77;
/// I2C address when SDO pin is pulled low.
pub const ADDR_SDO_LOW: u8 = 0x76;
const KT: f32 = 524288.0;  // temperature scale, 1x oversampling (unchanged)
const KP: f32 = 253952.0;  // pressure scale for 16x oversampling (was 524288 at 1x)

// ── Public types ──────────────────────────────────────────────────────────────

#[derive(Debug, defmt::Format, Clone, Copy)]
pub struct BaroData {
    pub pressure_pa:   f32,
    pub temperature_c: f32,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    I2c,
    WrongProductId(u8),
    NotReady,
    Timeout,
}

// ── Calibration coefficients ──────────────────────────────────────────────────

#[derive(Clone)]
struct Coef {
    c0: f32, c1: f32,
    c00: f32, c10: f32, c01: f32, c11: f32,
    c20: f32, c21: f32, c30: f32,
}

// ── Driver ────────────────────────────────────────────────────────────────────

pub struct Dps310I2c<I2C> {
    i2c:  I2C,
    addr: u8,
    coef: Option<Coef>,
}

impl<I2C: I2c> Dps310I2c<I2C> {
    /// Create a new driver. `addr` is 0x77 (SDO high) or 0x76 (SDO low).
    pub fn new(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr, coef: None }
    }

    // ── Low-level helpers ─────────────────────────────────────────────────────

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c
            .write_read(self.addr, &[reg], buf)
            .await
            .map_err(|_| Error::I2c)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read_regs(reg, &mut buf).await?;
        Ok(buf[0])
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c
            .write(self.addr, &[reg, val])
            .await
            .map_err(|_| Error::I2c)
    }

    // ── Calibration ───────────────────────────────────────────────────────────

    async fn read_coef(&mut self) -> Result<Coef, Error> {
        let mut r = [0u8; 18];
        self.read_regs(REG_COEF, &mut r).await?;

        fn sign_ext(v: i32, bits: u32) -> i32 {
            let shift = 32 - bits;
            (v << shift) >> shift
        }

        let c0  = sign_ext(((r[0] as i32) << 4) | ((r[1] as i32) >> 4), 12);
        let c1  = sign_ext((((r[1] as i32) & 0x0F) << 8) | (r[2] as i32), 12);
        let c00 = sign_ext(
            ((r[3] as i32) << 12) | ((r[4] as i32) << 4) | ((r[5] as i32) >> 4), 20);
        let c10 = sign_ext(
            (((r[5] as i32) & 0x0F) << 16) | ((r[6] as i32) << 8) | (r[7] as i32), 20);
        let c01 = i16::from_be_bytes([r[8],  r[9]])  as i32;
        let c11 = i16::from_be_bytes([r[10], r[11]]) as i32;
        let c20 = i16::from_be_bytes([r[12], r[13]]) as i32;
        let c21 = i16::from_be_bytes([r[14], r[15]]) as i32;
        let c30 = i16::from_be_bytes([r[16], r[17]]) as i32;

        defmt::info!("DPS310 coef: c0={} c1={} c00={} c10={}", c0, c1, c00, c10);

        Ok(Coef {
            c0: c0 as f32,   c1: c1 as f32,
            c00: c00 as f32, c10: c10 as f32, c01: c01 as f32,
            c11: c11 as f32, c20: c20 as f32, c21: c21 as f32,
            c30: c30 as f32,
        })
    }

    // ── Public API ────────────────────────────────────────────────────────────

    pub async fn init(&mut self) -> Result<(), Error> {
        self.write_reg(REG_RESET, 0x09).await?;
        Timer::after_millis(40).await;

        let id = self.read_reg(REG_PRODUCT_ID).await?;
        if id >> 4 != PRODUCT_ID >> 4 {
            return Err(Error::WrongProductId(id));
        }
        defmt::info!("DPS310: product_id=0x{:02X} ok", id);

        for _ in 0..20 {
            let meas = self.read_reg(REG_MEAS_CFG).await?;
            if meas & 0xC0 == 0xC0 { break; }
            Timer::after_millis(10).await;
        }
        if self.read_reg(REG_MEAS_CFG).await? & 0xC0 != 0xC0 {
            return Err(Error::NotReady);
        }

        self.coef = Some(self.read_coef().await?);

        // PM_RATE=101 (32 Hz), PM_PRC=0100 (16x oversampling) -> 0x54
        // 16x reduces RMS noise from ~3 Pa to ~0.3 Pa (~0.025 m altitude RMS).
        self.write_reg(REG_PRS_CFG, 0x54).await?;
        // TMP_EXT=1, TMP_RATE=101 (32 Hz), TMP_PRC=0000 (1x) -> 0xD0
        self.write_reg(REG_TMP_CFG, 0xD0).await?;
        // P_SHIFT (bit 2) required when PM_PRC > 3 (see datasheet Table 9).
        self.write_reg(REG_CFG_REG, 0x04).await?;
        self.write_reg(REG_MEAS_CFG, 0x07).await?; // continuous pressure + temperature

        defmt::info!("DPS310: init complete");
        Ok(())
    }

    pub async fn read(&mut self) -> Result<BaroData, Error> {
        let coef = self.coef.as_ref().ok_or(Error::NotReady)?.clone();

        for _ in 0..50 {
            if self.read_reg(REG_MEAS_CFG).await? & 0x60 == 0x60 { break; }
            Timer::after_millis(20).await;
        }
        if self.read_reg(REG_MEAS_CFG).await? & 0x60 != 0x60 {
            return Err(Error::Timeout);
        }

        let mut buf = [0u8; 6];
        self.read_regs(REG_PSR_B2, &mut buf).await?;

        let praw = sign_extend_24(
            ((buf[0] as i32) << 16) | ((buf[1] as i32) << 8) | (buf[2] as i32));
        let traw = sign_extend_24(
            ((buf[3] as i32) << 16) | ((buf[4] as i32) << 8) | (buf[5] as i32));

        let traw_sc = traw as f32 / KT;
        let praw_sc = praw as f32 / KP;

        let t = coef.c0 * 0.5 + coef.c1 * traw_sc;
        let p = coef.c00
            + praw_sc * (coef.c10 + praw_sc * (coef.c20 + praw_sc * coef.c30))
            + traw_sc * coef.c01
            + traw_sc * praw_sc * (coef.c11 + praw_sc * coef.c21);

        Ok(BaroData { pressure_pa: p, temperature_c: t })
    }
}

#[inline]
fn sign_extend_24(v: i32) -> i32 { (v << 8) >> 8 }
