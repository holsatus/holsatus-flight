use super::buffer::{Consumer, GrantR, Producer};
use embedded_io::ReadExactError;
use embedded_io_async::{BufRead, ErrorKind, ErrorType, Read, Write};

use super::{grant::GrantWriter, SerialState};

pub struct SoftWriter<'a> {
    pub(super) producer: Producer<'a>,
    pub(super) state: &'a SerialState,
}

impl SoftWriter<'_> {
    pub async fn grant_writer(&mut self) -> Result<GrantWriter<'_>, ErrorKind> {
        loop {
            let subscription = self.state.wait_writer.subscribe().await;

            if let Some(error) = self.state.error.take() {
                return Err(error);
            }

            match self.producer.grant_max_remaining() {
                Ok(grant_inner) => {
                    return Ok(GrantWriter {
                        state: self.state,
                        inner: grant_inner,
                    });
                }

                Err(super::buffer::Error::GrantInProgress) => {
                    panic!("Double-grants are illegal!");
                }

                // No bytes available to read, wait for subscription
                Err(super::buffer::Error::InsufficientSize) => {
                    let res = subscription.await;
                    debug_assert!(res.is_ok());
                }
            }
        }
    }

    pub async fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        self.grant_writer()
            .await
            .map(|grant| grant.copy_max_from(buf))
    }

    pub async fn flush(&mut self) -> Result<(), ErrorKind> {
        Ok(()) // TODO: Imlpement forwarding flush
    }

    pub async fn write_all(&mut self, buf: &[u8]) {
        _ = <Self as Write>::write_all(self, buf).await
    }
}

pub struct SoftReader<'a> {
    pub(super) consumer: Consumer<'a>,
    pub(super) state: &'a SerialState,
    pub(super) grant: Option<GrantR<'a>>,
}

impl<'a> SoftReader<'a> {
    async fn get_grant(&mut self) -> Result<&mut GrantR<'a>, ErrorKind> {
        loop {
            let subscription = self.state.wait_reader.subscribe().await;

            if let Some(error) = self.state.error.take() {
                return Err(error);
            }

            // Release any existing grant, double-grants are illegal anyway
            self.grant = None;

            match self.consumer.read() {
                Ok(g) => return Ok(self.grant.insert(g)),
                _ => _ = subscription.await,
            }
        }
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        let available = self.fill_buf().await?;
        let bytes = buf.len().min(available.len());
        buf[..bytes].copy_from_slice(&available[..bytes]);
        self.consume(bytes);

        Ok(bytes)
    }

    async fn fill_buf(&mut self) -> Result<&[u8], ErrorKind> {
        self.get_grant().await.map(|grant| grant.buf())
    }

    fn consume(&mut self, amt: usize) {
        if let Some(grant) = self.grant.take() {
            grant.release(amt);
        }
    }
}

// ---- embedded-io-async impls ---- //

impl ErrorType for SoftWriter<'_> {
    type Error = ErrorKind;
}

impl Write for SoftWriter<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        SoftWriter::write(self, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        SoftWriter::flush(self).await
    }
}

impl ErrorType for SoftReader<'_> {
    type Error = ErrorKind;
}

impl Read for SoftReader<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        SoftReader::read(self, buf).await
    }
}

impl BufRead for SoftReader<'_> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        SoftReader::fill_buf(self).await
    }

    fn consume(&mut self, amt: usize) {
        SoftReader::consume(self, amt)
    }
}

// ---------------

#[allow(async_fn_in_trait)]
pub trait BufReadExt: BufRead {
    async fn skip_until(&mut self, delim: u8) -> Result<usize, ReadExactError<Self::Error>> {
        let mut read: usize = 0;
        loop {
            let (done, used) = {
                let available = self.fill_buf().await?;

                if available.is_empty() {
                    return Err(ReadExactError::UnexpectedEof);
                }

                match available.iter().position(|p| *p == delim) {
                    Some(i) => (true, i + 1),
                    None => (false, available.len()),
                }
            };
            self.consume(used);
            read += used;
            if done || used == 0 {
                return Ok(read);
            }
        }
    }
}

impl<R: BufRead> BufReadExt for R {}

mod test {

    #[test]
    fn test_buf_read_skip_until() {
        use crate::{new_serial_reader, sync::serial::soft::BufReadExt};
        use embedded_io_async::Write;

        const BUF: &[u8] = b"dingus_foo_bar_baz";

        let mut reader = new_serial_reader!(20);

        futures_executor::block_on(async {
            reader.hardware.write_all(BUF.as_ref()).await.unwrap();

            let mut reader = reader.software;

            assert_eq!(reader.fill_buf().await.unwrap(), BUF);

            reader.consume(7);

            assert_eq!(reader.fill_buf().await.unwrap(), b"foo_bar_baz");

            assert_eq!(reader.skip_until(b"_"[0]).await.unwrap(), 4);

            assert_eq!(reader.fill_buf().await.unwrap(), b"bar_baz");

            assert_eq!(reader.skip_until(b"_"[0]).await.unwrap(), 4);

            assert_eq!(reader.fill_buf().await.unwrap(), b"baz");
        })
    }

    #[test]
    fn test_buf_read() {
        use crate::new_serial_reader;
        use embedded_io_async::Write;

        const BUF: &[u8] = b"_foo_bar_baz";

        let mut reader = new_serial_reader!(20);

        futures_executor::block_on(async {
            reader.hardware.write_all(BUF.as_ref()).await.unwrap();

            let mut reader = reader.software;

            let mut buf = [0u8; 4];
            assert_eq!(reader.read(buf.as_mut()).await.unwrap(), 4);
            assert_eq!(&buf, b"_foo");

            let mut buf = [0u8; 4];
            assert_eq!(reader.read(buf.as_mut()).await.unwrap(), 4);
            assert_eq!(&buf, b"_bar");

            let mut buf = [0u8; 4];
            assert_eq!(reader.read(buf.as_mut()).await.unwrap(), 4);
            assert_eq!(&buf, b"_baz");
        })
    }
}
