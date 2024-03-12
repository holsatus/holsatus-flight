
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

// Implement a buffer writer for testing purposes. This is a simple buffer that can be written to and then read from.

pub(super) mod writer_buffer {
    use embedded_io::ErrorKind;

    pub struct BufWriter<const N: usize> {
        buff: [u8; N],
        read: usize
    }

    impl <const N: usize> embedded_io::ErrorType for BufWriter<N> {
        type Error = ErrorKind;
    }

    impl <const N: usize> BufWriter<N> {
        #[allow(unused)]
        pub fn new() -> Self {
            Self {
                buff: [0; N],
                read: 0,
            }
        }

        // Read the contents of this buffer
        #[allow(unused)]
        pub fn empty(&mut self) -> &[u8] {
            let con = &self.buff[..self.read];
            self.read = 0;
            con
        }
    }

    impl <const N: usize> embedded_io::Write for BufWriter<N> {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let len = buf.len();
            if len + self.read > N {
                return Err(ErrorKind::OutOfMemory);
            }
            self.buff[self.read..(len + self.read)].copy_from_slice(buf);
            self.read += len;
            Ok(len)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}