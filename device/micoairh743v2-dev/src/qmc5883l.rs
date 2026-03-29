//! QMC5883L magnetometer driver (I2C, address 0x0D).
//!
//! Minimal driver: init + single read of X/Y/Z in continuous mode.
//! Sensitivity: 0.244 uT/LSB (2 Gauss range).

use embedded_hal_async::i2c::I2c;

pub const ADDR: u8 = 0x0D;

const REG_DATA:   u8 = 0x00; // X_LSB..Z_MSB (6 bytes)
const REG_STATUS: u8 = 0x06;
const REG_CTRL1:  u8 = 0x09; // mode, ODR, RNG, OSR
const REG_CTRL2:  u8 = 0x0A; // soft reset
const REG_SRST:   u8 = 0x0B; // SET/RESET period

pub struct MagData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, defmt::Format)]
pub enum Error {
    I2c,
}

pub struct Qmc5883l<I2C> {
    i2c: I2C,
}

impl<I2C: I2c> Qmc5883l<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        // Soft reset.
        self.write_reg(REG_CTRL2, 0x80).await?;
        embassy_time::Timer::after_millis(10).await;

        // SET/RESET period = 0x01 (recommended by datasheet).
        self.write_reg(REG_SRST, 0x01).await?;

        // CTRL1: [7:6]=OSR=512(00), [5:4]=RNG=2G(00), [3:2]=ODR=200Hz(11), [1:0]=Continuous(01)
        self.write_reg(REG_CTRL1, 0b0000_1101).await?;

        Ok(())
    }

    /// Read X/Y/Z raw counts (2G range, 0.244 uT/LSB).
    pub async fn read(&mut self) -> Result<MagData, Error> {
        // Poll DRDY (status bit 0).
        for _ in 0..10 {
            let mut status = [0u8; 1];
            self.i2c
                .write_read(ADDR, &[REG_STATUS], &mut status)
                .await
                .map_err(|_| Error::I2c)?;
            if status[0] & 0x01 != 0 {
                break;
            }
            embassy_time::Timer::after_millis(1).await;
        }

        let mut buf = [0u8; 6];
        self.i2c
            .write_read(ADDR, &[REG_DATA], &mut buf)
            .await
            .map_err(|_| Error::I2c)?;

        Ok(MagData {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        })
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c.write(ADDR, &[reg, val]).await.map_err(|_| Error::I2c)
    }
}
