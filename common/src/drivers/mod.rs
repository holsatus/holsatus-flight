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

struct TdkSpiTransfer<TRANSFER> {
    pub transfer: TRANSFER,
}

// Implementation of register device trait for Spi
impl<SPI: embedded_hal_async::spi::SpiDevice> RegisterTransfer for TdkSpiTransfer<SPI> {
    type Error = SPI::Error;
    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer.transaction(&mut [
            embedded_hal_async::spi::Operation::Write(&[reg_addr | 0x80]),
            embedded_hal_async::spi::Operation::Read(read),
        ])
        .await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.transfer.transaction(&mut [
            embedded_hal_async::spi::Operation::Write(&[reg_addr]),
            embedded_hal_async::spi::Operation::Write(write),
        ])
        .await
    }
}

struct TdkI2cTransfer<BUS> {
    pub bus: BUS,
    pub addr: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> RegisterTransfer for TdkI2cTransfer<I2C> {
    type Error = I2C::Error;

    async fn read_registers(&mut self, reg_addr: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.transaction(
            self.addr,
            &mut [
            embedded_hal_async::i2c::Operation::Write(&[reg_addr | 0x80]),
            embedded_hal_async::i2c::Operation::Read(read),
        ])
        .await
    }

    async fn write_registers(&mut self, reg_addr: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.bus.transaction(
            self.addr,
            &mut [
            embedded_hal_async::i2c::Operation::Write(&[reg_addr]),
            embedded_hal_async::i2c::Operation::Write(write),
        ])
        .await
    }
}

