pub mod imu;

/// Represents a device which operates on reading and writing a series of bytes
/// from registers. Typically this would include I2C and SPI devices.
#[allow(async_fn_in_trait)]
pub trait RegisterTransfer {
    type Error;
    async fn read_registers(
        &mut self,
        reg_addr: u8,
        read: &mut [u8],
    ) -> Result<(), Self::Error>;
    async fn write_registers(
        &mut self,
        reg_addr: u8,
        write: &[u8],
    ) -> Result<(), Self::Error>;
}

pub struct TdkSpiTransfer<SPI> {
    pub spi: SPI,
}

// Implementation of register device trait for Spi
impl<Spi: embedded_hal_async::spi::SpiDevice> RegisterTransfer for TdkSpiTransfer<Spi> {
    type Error = Spi::Error;
    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [
            embedded_hal_async::spi::Operation::Write(&[reg_addr | 0x80]),
            embedded_hal_async::spi::Operation::Read(read),
        ])
        .await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [
            embedded_hal_async::spi::Operation::Write(&[reg_addr]),
            embedded_hal_async::spi::Operation::Write(write),
        ])
        .await
    }
}

pub struct TdkI2cTransfer<I2C> {
    pub i2c: I2C,
    pub addr: u8,
}

impl<I2c: embedded_hal_async::i2c::I2c> RegisterTransfer for TdkI2cTransfer<I2c> {
    type Error = I2c::Error;

    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.transaction(
            self.addr,
            &mut [
            embedded_hal_async::i2c::Operation::Write(&[reg_addr | 0x80]),
            embedded_hal_async::i2c::Operation::Read(read),
        ])
        .await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.i2c.transaction(
            self.addr,
            &mut [
            embedded_hal_async::i2c::Operation::Write(&[reg_addr]),
            embedded_hal_async::i2c::Operation::Write(write),
        ])
        .await
    }
}

