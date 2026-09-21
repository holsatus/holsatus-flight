use embedded_hal::i2c::Error as _;
use futures::TryFutureExt as _;

/// Newtype wrapper for I2c busses, using the non-generic ErrorKind
pub struct WrappedI2c<I2c>(pub I2c);

impl<I2c> embedded_hal_async::i2c::ErrorType for WrappedI2c<I2c> {
    type Error = embedded_hal_async::i2c::ErrorKind;
}

impl<I2c> embedded_hal_async::i2c::I2c for WrappedI2c<I2c>
where
    I2c: embedded_hal_async::i2c::I2c,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.0
            .transaction(address, operations)
            .map_err(|error| error.kind())
    }
}
