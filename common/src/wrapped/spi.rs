use embedded_hal::spi::Error as _;
use futures::TryFutureExt;

/// Newtype wrapper for Spi devices, using the non-generic ErrorKind
pub struct WrappedSpi<Spi>(pub Spi);

impl<Spi> embedded_hal_async::spi::ErrorType for WrappedSpi<Spi> {
    type Error = embedded_hal_async::spi::ErrorKind;
}

impl<Spi> embedded_hal_async::spi::SpiDevice for WrappedSpi<Spi>
where
    Spi: embedded_hal_async::spi::SpiDevice,
{
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.0.transaction(operations).map_err(|error| error.kind())
    }
}
