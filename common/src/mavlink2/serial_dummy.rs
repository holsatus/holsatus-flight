pub struct DummySerialRx {}
pub struct DummySerialTx {}

impl embedded_io_async::ErrorType for DummySerialRx {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::ErrorType for DummySerialTx {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::Read for DummySerialRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        todo!()
    }
}

impl embedded_io_async::Write for DummySerialTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        todo!()
    }
}
