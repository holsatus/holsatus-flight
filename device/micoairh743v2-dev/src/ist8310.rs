//! Async IST8310 magnetometer driver over I2C.
//!
//! Generic over `embedded_hal_async::i2c::I2c`.
//!
//! I2C address: 0x0E (fixed).
//! Output range: +-1600 uT, sensitivity 0.3 uT/LSB.

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

// ── Register addresses ────────────────────────────────────────────────────────

const REG_WHOAMI:  u8 = 0x00;
const REG_STAT1:   u8 = 0x02;
const REG_DATA:    u8 = 0x03; // XL XH YL YH ZL ZH (6 bytes, little-endian)
const REG_CNTL1:   u8 = 0x0A; // measurement control
const REG_CNTL2:   u8 = 0x0B; // soft reset
const REG_AVGCNTL: u8 = 0x41; // averaging control
const REG_PDCNTL:  u8 = 0x42; // pulse duration

pub const ADDR: u8 = 0x0E;

const WHOAMI_VAL: u8 = 0x10;
const STAT1_DRDY: u8 = 0x01;

/// Sensitivity: 0.3 uT per LSB.
pub const SENSITIVITY_UT_PER_LSB: f32 = 0.3;

// ── Public types ──────────────────────────────────────────────────────────────

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct MagData {
    /// Raw counts, 0.3 uT/LSB.
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    I2c,
    WrongId(u8),
    Timeout,
}

// ── Driver ────────────────────────────────────────────────────────────────────

pub struct Ist8310<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> Ist8310<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c.write(ADDR, &[reg, val]).await.map_err(|_| Error::I2c)
    }

    async fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.i2c.write_read(ADDR, &[reg], buf).await.map_err(|_| Error::I2c)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read_regs(reg, &mut buf).await?;
        Ok(buf[0])
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Soft reset
        self.write_reg(REG_CNTL2, 0x01).await?;
        Timer::after_millis(10).await;

        let id = self.read_reg(REG_WHOAMI).await?;
        if id != WHOAMI_VAL {
            return Err(Error::WrongId(id));
        }
        defmt::info!("IST8310: whoami=0x{:02X} ok", id);

        // 2-sample averaging, normal pulse duration
        self.write_reg(REG_AVGCNTL, 0x09).await?;
        self.write_reg(REG_PDCNTL, 0xC0).await?;

        defmt::info!("IST8310: init complete");
        Ok(())
    }

    /// Trigger one single measurement and wait for data ready (max ~10 ms).
    pub async fn read(&mut self) -> Result<MagData, Error> {
        // Single measurement mode
        self.write_reg(REG_CNTL1, 0x01).await?;

        // Poll DRDY (bit 0 of STAT1), timeout after ~10 ms
        for _ in 0..20 {
            Timer::after_millis(1).await;
            if self.read_reg(REG_STAT1).await? & STAT1_DRDY != 0 {
                break;
            }
        }
        if self.read_reg(REG_STAT1).await? & STAT1_DRDY == 0 {
            return Err(Error::Timeout);
        }

        let mut buf = [0u8; 6];
        self.read_regs(REG_DATA, &mut buf).await?;

        Ok(MagData {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }
}
