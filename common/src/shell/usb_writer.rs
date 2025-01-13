
pub(super) mod writer_embassy_usb {
    use embassy_usb::{class::cdc_acm::Sender, driver::Driver};
    use embedded_io::ErrorKind;

    pub struct UsbWriter<'w, D: Driver<'w>> {
        tx: Sender<'w, D>
    }

    impl <'w, D: Driver<'w>> UsbWriter<'w, D> {
        pub fn new(tx: Sender<'w, D>) -> Self {
            Self {
                tx
            }
        }
    }

    impl <'w, D: Driver<'w>> embedded_io::ErrorType for UsbWriter<'w, D> {
        type Error = ErrorKind;
    }

    impl <'w, D: Driver<'w>> embedded_io_async::Write for UsbWriter<'w, D> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            trace!("Writing {} bytes to USB: {:?}", buf.len(), buf);
            // For now writing is converted to a blocking operation
            let mut written = 0;
            for chunk in buf.chunks(self.tx.max_packet_size() as usize) {
                match self.tx.write_packet(chunk).await {
                    Ok(_) => written += chunk.len(),
                    Err(_) if written > 0 => return Ok(written),
                    Err(_) => return Err(ErrorKind::NotConnected),
                }
            }
            Ok(written)
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}