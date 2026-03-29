//! Async BMI270 IMU driver — generic over `embedded_hal_async::spi::SpiDevice`.
//!
//! CS management is handled by the SpiDevice implementor, so this driver
//! works with both:
//!   - `SpiDeviceWithConfig` (shared bus, embassy-embedded-hal)
//!   - Any other async SpiDevice implementation
//!
//! # Usage
//! ```rust
//! let mut imu = Bmi270::new(spi_device);
//! imu.init(&BMI270_CONFIG_FILE).await.unwrap();
//! let data = imu.read().await.unwrap();
//! // data.accel.z ≈ 16384 flat (1 g at default ±2 g range)
//! ```

use embassy_time::Timer;
use embedded_hal_async::spi::{Operation, SpiDevice};

// ── Config blob (vendored from qrasmont/bmi2, Apache-2.0) ────────────────────
#[path = "vendor/bmi270_config.rs"]
mod config;
pub use config::BMI270_CONFIG_FILE;

// ── Register addresses ────────────────────────────────────────────────────────

const REG_CHIP_ID: u8 = 0x00;
const REG_ACC_DATA_X_LSB: u8 = 0x0C;
const REG_INTERNAL_STATUS: u8 = 0x21;
const REG_INIT_CTRL: u8 = 0x59;
const REG_INIT_ADDR_0: u8 = 0x5B;
const REG_INIT_DATA: u8 = 0x5E;
const REG_PWR_CONF: u8 = 0x7C;
const REG_PWR_CTRL: u8 = 0x7D;
const REG_CMD: u8 = 0x7E;

const CHIP_ID: u8 = 0x24;

// ── Public data types ─────────────────────────────────────────────────────────

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct AccelData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct GyroData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct ImuData {
    pub accel: AccelData,
    pub gyro: GyroData,
}

// ── Error type ────────────────────────────────────────────────────────────────

#[derive(Debug, defmt::Format)]
pub enum Error {
    Spi,
    WrongChipId(u8),
    InitFailed { status: u8 },
}

// ── Driver ────────────────────────────────────────────────────────────────────

pub struct Bmi270<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Bmi270<SPI> {
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    // ── Low-level helpers ─────────────────────────────────────────────────────

    /// Read one register.
    /// BMI270 SPI protocol: send [0x80|reg, dummy] → receive [garbage, dummy, data].
    /// We use transfer_in_place with 3 bytes; data is in position [2].
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0x80 | reg, 0x00, 0x00];
        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(|_| Error::Spi)?;
        Ok(buf[2])
    }

    /// Write one register.
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.spi
            .write(&[reg & 0x7F, val])
            .await
            .map_err(|_| Error::Spi)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Initialise the sensor and upload the configuration blob.
    /// Takes ~200 ms.
    pub async fn init(&mut self, config: &[u8]) -> Result<(), Error> {
        // 0. Probe chip ID; only soft-reset if it looks wrong.
        //    After DFU flash the sensor is already live and responds correctly —
        //    an unconditional reset delays startup and can leave the SPI state
        //    machine confused if the MCU rebooted mid-transaction.
        //    After an ESC power cycle (sensor lost power, MCU stayed USB-powered)
        //    the ID will be garbage → we reset and retry.
        let needs_reset = match self.read_reg(REG_CHIP_ID).await {
            Ok(id) if id == CHIP_ID => false,
            _ => true,
        };
        if needs_reset {
            self.write_reg(REG_CMD, 0xB6).await?; // soft reset
            Timer::after_millis(10).await;
        }

        // 1. Verify chip ID
        let id = self.read_reg(REG_CHIP_ID).await?;
        if id != CHIP_ID {
            return Err(Error::WrongChipId(id));
        }
        //defmt::info!("BMI270: chip_id=0x{:02X} ok", id);

        // 2. Exit advanced power-save
        self.write_reg(REG_PWR_CONF, 0x00).await?;
        Timer::after_micros(450).await;

        // 3. Halt config loading
        self.write_reg(REG_INIT_CTRL, 0x00).await?;

        // 4. Burst-upload config blob in 256-byte chunks
        for (i, chunk) in config.chunks(256).enumerate() {
            let word_addr = i * 128;
            let addr_lo = (word_addr & 0x0F) as u8;
            let addr_hi = ((word_addr >> 4) & 0xFF) as u8;

            // Write word address
            self.spi
                .write(&[REG_INIT_ADDR_0 & 0x7F, addr_lo, addr_hi])
                .await
                .map_err(|_| Error::Spi)?;

            // Burst-write chunk — single atomic transaction keeps CS asserted
            self.spi
                .transaction(&mut [
                    Operation::Write(&[REG_INIT_DATA & 0x7F]),
                    Operation::Write(chunk),
                ])
                .await
                .map_err(|_| Error::Spi)?;
        }

        // 5. Trigger init
        self.write_reg(REG_INIT_CTRL, 0x01).await?;

        // 6. Poll INTERNAL_STATUS bit 0
        Timer::after_millis(150).await;
        let status = self.read_reg(REG_INTERNAL_STATUS).await?;
        if status & 0x01 == 0 {
            return Err(Error::InitFailed { status });
        }
        //defmt::info!("BMI270: INTERNAL_STATUS=0x{:02X} ok", status);

        // 7. Enable accel + gyro + temp
        self.write_reg(REG_PWR_CTRL, 0x0E).await?;

        // 8. Normal power mode
        self.write_reg(REG_PWR_CONF, 0x02).await?;
        Timer::after_millis(100).await;

        Ok(())
    }

    /// Read one accelerometer + gyroscope sample.
    pub async fn read(&mut self) -> Result<ImuData, Error> {
        // 14-byte transfer: [addr, dummy, ax_l, ax_h, ay_l, ay_h, az_l, az_h,
        //                          gx_l, gx_h, gy_l, gy_h, gz_l, gz_h]
        let mut buf = [0u8; 14];
        buf[0] = 0x80 | REG_ACC_DATA_X_LSB;
        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(|_| Error::Spi)?;

        // buf[0] = garbage (response to addr byte)
        // buf[1] = dummy pipeline byte
        // buf[2..14] = 12 data bytes
        Ok(ImuData {
            accel: AccelData {
                x: i16::from_le_bytes([buf[2], buf[3]]),
                y: i16::from_le_bytes([buf[4], buf[5]]),
                z: i16::from_le_bytes([buf[6], buf[7]]),
            },
            gyro: GyroData {
                x: i16::from_le_bytes([buf[8], buf[9]]),
                y: i16::from_le_bytes([buf[10], buf[11]]),
                z: i16::from_le_bytes([buf[12], buf[13]]),
            },
        })
    }
}
