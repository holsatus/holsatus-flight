
// Implementing wrapper class for the CdcAcmClass Sender, to support embedded_io::Write.
// These operations are not async, so we use block_on to convert them async to blocking.

pub(super) mod writer_embassy_usb {
    use embassy_usb::{class::cdc_acm::Sender, driver::{Driver as UsbDriver, EndpointError}};
    use embedded_io::ErrorKind;

    pub struct UsbWriter<'w, D: UsbDriver<'w>> {
        tx: Sender<'w, D>
    }

    impl <'w, D: UsbDriver<'w>> UsbWriter<'w, D> {
        pub fn new(tx: Sender<'w, D>) -> Self {
            Self {
                tx
            }
        }
    }

    impl <'w, D: UsbDriver<'w>> embedded_io::ErrorType for UsbWriter<'w, D> {
        type Error = ErrorKind;
    }

    impl <'w, D: UsbDriver<'w>> embedded_io::Write for UsbWriter<'w, D> {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            defmt::trace!("Writing {} bytes to USB: {:?}", buf.len(), buf);
            // For now writing is converted to a blocking operation
            embassy_futures::block_on(async {
                for chunk in buf.chunks(self.tx.max_packet_size() as usize) {
                    self.tx.write_packet(chunk).await.map_err(|e| {
                        match e {
                            EndpointError::BufferOverflow => ErrorKind::OutOfMemory,
                            EndpointError::Disabled => ErrorKind::NotConnected,
                        }
                    })?;
                }
                Ok(())
            }).map(|_| {
                buf.len()
            })
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

}
