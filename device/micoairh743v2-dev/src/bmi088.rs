//! Async BMI088 / BMI088_MM IMU driver (accel + gyro).
//!
//! The `_mm_` suffix (Motor-Module) is Bosch's industrial/drone variant of the
//! BMI088 with extended availability.  Init sequence and register map are
//! identical; the only difference is the accel chip-ID byte (0x1A vs 0x1E).
//! Both are accepted.
//!
//! # Hardware
//!   Accel CS = PD4  --  SPI read inserts one dummy byte after the address
//!   Gyro  CS = PD5  --  SPI read has NO dummy byte
//!
//! # SPI protocols (from Bosch SensorAPI bmi08a.c / bmi08g.c)
//!   Accel read : send [0x80|reg, 0x00, 0x00, ...]  data starts at byte 2
//!   Accel write: send [reg & 0x7F, val]
//!   Gyro  read : send [0x80|reg, 0x00, ...]        data starts at byte 1
//!   Gyro  write: send [reg & 0x7F, val]
//!
//! # Scales (configured below)
//!   Accel: +-6 g,    1 g   = 5460.8 LSB  (ACC_RANGE = 0x01)
//!   Gyro:  +-2000 dps, 1 dps = 16.384 LSB (GYR_RANGE = 0x00)

use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;

// ── Accel register map ────────────────────────────────────────────────────────

const ACC_CHIP_ID:   u8 = 0x00;
const ACC_X_LSB:     u8 = 0x12;
const ACC_CONF:      u8 = 0x40; // [7:4] BW  [3:0] ODR
const ACC_RANGE:     u8 = 0x41;
const ACC_PWR_CONF:  u8 = 0x7C; // 0x00 = active, 0x03 = suspend
const ACC_PWR_CTRL:  u8 = 0x7D; // 0x04 = enable, 0x00 = disable
const ACC_SOFTRESET: u8 = 0x7E; // write 0xB6

// ACC_CONF = (BW << 4) | ODR
//   BW  0x0A = normal filter
//   ODR 0x0C = 1600 Hz
const ACC_CONF_VAL:  u8 = (0x0A << 4) | 0x0C; // 0xAC
const ACC_RANGE_6G:  u8 = 0x01;

// ── Gyro register map ─────────────────────────────────────────────────────────

const GYR_CHIP_ID:   u8 = 0x00;
const GYR_X_LSB:     u8 = 0x02;
const GYR_RANGE:     u8 = 0x0F; // 0x00 = +-2000 dps
const GYR_BW:        u8 = 0x10; // 0x01 = 2000 Hz ODR, 230 Hz filter
const GYR_SOFTRESET: u8 = 0x14; // write 0xB6

// ── Chip IDs ──────────────────────────────────────────────────────────────────

/// BMI088 accel (standard)
const ACCEL_ID_BMI088:    u8 = 0x1E;
/// BMI088_MM accel (motor-module / industrial variant)
const ACCEL_ID_BMI088_MM: u8 = 0x1A;
/// BMI085 accel (accepted as compatible)
const ACCEL_ID_BMI085:    u8 = 0x1F;
/// Gyro chip ID (same for all variants)
const GYRO_ID: u8 = 0x0F;

// ── Public types ──────────────────────────────────────────────────────────────

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct AccelData { pub x: i16, pub y: i16, pub z: i16 }

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct GyroData  { pub x: i16, pub y: i16, pub z: i16 }

#[derive(Debug, defmt::Format, Clone, Copy, Default)]
pub struct ImuData   { pub accel: AccelData, pub gyro: GyroData }

#[derive(Debug, defmt::Format)]
pub enum Error {
    Spi,
    WrongAccelId(u8),
    WrongGyroId(u8),
}

// ── Driver ────────────────────────────────────────────────────────────────────

pub struct Bmi088<ACCEL, GYRO> {
    accel: ACCEL,
    gyro:  GYRO,
}

impl<ACCEL: SpiDevice, GYRO: SpiDevice> Bmi088<ACCEL, GYRO> {
    pub fn new(accel: ACCEL, gyro: GYRO) -> Self {
        Self { accel, gyro }
    }

    // ── Accel register access (dummy byte after address) ──────────────────────

    async fn a_read(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0x80 | reg, 0x00, 0x00];
        self.accel.transfer_in_place(&mut buf).await.map_err(|_| Error::Spi)?;
        Ok(buf[2])
    }

    async fn a_write(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.accel.write(&[reg & 0x7F, val]).await.map_err(|_| Error::Spi)
    }

    // ── Gyro register access (no dummy byte) ──────────────────────────────────

    async fn g_read(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0x80 | reg, 0x00];
        self.gyro.transfer_in_place(&mut buf).await.map_err(|_| Error::Spi)?;
        Ok(buf[1])
    }

    async fn g_write(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.gyro.write(&[reg & 0x7F, val]).await.map_err(|_| Error::Spi)
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Initialise both accel and gyro.
    ///
    /// Follows the Bosch SensorAPI (bmi08a.c / bmi08g.c) sequence:
    ///   - Accel: dummy read to activate SPI, soft-reset, dummy read after
    ///     reset (mandatory per Bosch), verify chip ID, enable power, configure.
    ///   - Gyro:  soft-reset, wait 30 ms, verify chip ID, configure.
    pub async fn init(&mut self) -> Result<(), Error> {
        // ── Accel ──────────────────────────────────────────────────────────────
        // One dummy read: toggles CS and switches the accel from I2C
        // auto-detect into SPI mode permanently.
        let _ = self.a_read(ACC_CHIP_ID).await;
        Timer::after_micros(500).await;

        self.a_write(ACC_SOFTRESET, 0xB6).await?;
        Timer::after_millis(1).await;

        // Mandatory dummy read after soft-reset over SPI (Bosch bmi08a.c L654).
        let _ = self.a_read(ACC_CHIP_ID).await;

        let acc_id = self.a_read(ACC_CHIP_ID).await?;
        if !matches!(acc_id, ACCEL_ID_BMI088 | ACCEL_ID_BMI088_MM | ACCEL_ID_BMI085) {
            return Err(Error::WrongAccelId(acc_id));
        }

        // Active mode, then enable (5 ms between each per Bosch).
        self.a_write(ACC_PWR_CONF, 0x00).await?;
        Timer::after_millis(5).await;
        self.a_write(ACC_PWR_CTRL, 0x04).await?;
        Timer::after_millis(5).await;

        self.a_write(ACC_CONF,  ACC_CONF_VAL).await?;
        self.a_write(ACC_RANGE, ACC_RANGE_6G).await?;

        // ── Gyro ───────────────────────────────────────────────────────────────
        self.g_write(GYR_SOFTRESET, 0xB6).await?;
        Timer::after_millis(30).await; // Bosch: BMI08_GYRO_SOFTRESET_DELAY = 30 ms

        let gyr_id = self.g_read(GYR_CHIP_ID).await?;
        if gyr_id != GYRO_ID {
            return Err(Error::WrongGyroId(gyr_id));
        }

        self.g_write(GYR_RANGE, 0x00).await?; // +-2000 dps
        self.g_write(GYR_BW,    0x01).await?; // 2000 Hz ODR, 230 Hz filter

        Ok(())
    }

    /// Read one accel + gyro sample.
    ///
    /// Accel: burst-reads 6 bytes from ACC_X_LSB with the mandatory dummy byte.
    /// Gyro:  burst-reads 6 bytes from GYR_X_LSB without a dummy byte.
    pub async fn read(&mut self) -> Result<ImuData, Error> {
        // [addr, dummy, ax_l, ax_h, ay_l, ay_h, az_l, az_h]
        let mut a = [0u8; 8];
        a[0] = 0x80 | ACC_X_LSB;
        self.accel.transfer_in_place(&mut a).await.map_err(|_| Error::Spi)?;

        // [addr, gx_l, gx_h, gy_l, gy_h, gz_l, gz_h]
        let mut g = [0u8; 7];
        g[0] = 0x80 | GYR_X_LSB;
        self.gyro.transfer_in_place(&mut g).await.map_err(|_| Error::Spi)?;

        Ok(ImuData {
            accel: AccelData {
                x: i16::from_le_bytes([a[2], a[3]]),
                y: i16::from_le_bytes([a[4], a[5]]),
                z: i16::from_le_bytes([a[6], a[7]]),
            },
            gyro: GyroData {
                x: i16::from_le_bytes([g[1], g[2]]),
                y: i16::from_le_bytes([g[3], g[4]]),
                z: i16::from_le_bytes([g[5], g[6]]),
            },
        })
    }
}
