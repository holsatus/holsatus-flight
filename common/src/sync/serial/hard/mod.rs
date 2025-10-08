use super::bbbuffer::{Consumer, Producer};
use embedded_io_async::{ErrorType, Read, Write};

use super::{
    grant::{GrantReader, GrantWriter},
    SerialState,
};

pub struct HardWriter<'a> {
    pub(super) producer: Producer<'a>,
    pub(super) state: &'a SerialState,
}

impl HardWriter<'_> {
    pub async fn grant_writer(&mut self) -> GrantWriter<'_> {
        loop {
            let subscribtion = self.state.wait_writer.subscribe().await;
            match self.producer.grant_max_remaining() {
                Ok(grant_inner) => {
                    return GrantWriter {
                        state: self.state,
                        inner: grant_inner,
                    }
                }

                Err(super::bbbuffer::Error::GrantInProgress) => {
                    panic!("Double-grants are illegal!");
                }

                _ => {
                    let res = subscribtion.await;
                    debug_assert!(res.is_ok());
                }
            }
        }
    }

    pub fn insert_error(&mut self, error: impl embedded_io_async::Error) {
        self.state.error.set(error.kind());
        self.state.wait_reader.wake();
    }

    pub async fn write(&mut self, buf: &[u8]) -> usize {
        self.grant_writer().await.copy_max_from(buf)
    }

    /// Connect this writer to another [`embedded_io_async::Read`], such that
    /// all bytes received through `reader` will be copied to this writers buffer.
    ///
    /// This will loop forever, or until the reader returns an EOF conditions,
    /// represented by a 0 byte read.
    pub async fn connect<R: Read>(&mut self, mut reader: R) {
        loop {
            let mut grant = self.grant_writer().await;
            match reader.read(grant.buffer_mut()).await {
                Ok(0) => break, // This should not happen
                Ok(bytes) => grant.commit(bytes),
                Err(error) => {
                    drop(grant);
                    self.insert_error(error)
                }
            }
        }
    }
}

pub struct HardReader<'a> {
    pub(super) consumer: Consumer<'a>,
    pub(super) state: &'a SerialState,
}

impl HardReader<'_> {
    pub async fn grant_reader(&mut self) -> GrantReader<'_> {
        loop {
            let subscribtion = self.state.wait_reader.subscribe().await;
            match self.consumer.read() {
                Ok(grant_inner) => {
                    return GrantReader {
                        state: self.state,
                        inner: grant_inner,
                    }
                }

                Err(super::bbbuffer::Error::GrantInProgress) => {
                    panic!("Double-grants are illegal!");
                }

                _ => {
                    let res = subscribtion.await;
                    debug_assert!(res.is_ok());
                }
            }
        }
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> usize {
        self.grant_reader().await.copy_max_into(buf)
    }

    pub fn insert_error(&mut self, error: impl embedded_io_async::Error) {
        self.state.error.set(error.kind());
        self.state.wait_writer.wake();
    }

    pub async fn read_exact(&mut self, buf: &mut [u8]) {
        let res = Read::read_exact(self, buf).await;

        // Even though our implementation is infallible, read_exact *could* encounter
        // an EOF conditions, if the number of bytes received is 0. This should never
        // happen, since bbqueue should never give us an empty grant. Use debug assert
        // here to catch any regression.
        debug_assert!(res.is_ok());
    }

    /// Connect this reader to another [`embedded_io_async::Write`], such that
    /// all bytes received through this reader will be written to `writer`.
    ///
    /// This will loop forever, *or* until the `writer` reaches an EOF condition.
    pub async fn connect<W: Write>(&mut self, mut writer: W) {
        loop {
            let grant = self.grant_reader().await;
            match writer.write(grant.buffer()).await {
                Ok(0) => break,
                Ok(bytes) => grant.release(bytes),
                Err(error) => {
                    drop(grant);
                    self.insert_error(error)
                }
            }
        }
    }
}

// ---- embedded-io-async impls ---- //

impl ErrorType for HardReader<'_> {
    type Error = core::convert::Infallible;
}

impl Write for HardWriter<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(HardWriter::write(self, buf).await)
    }
}

impl ErrorType for HardWriter<'_> {
    type Error = core::convert::Infallible;
}

impl Read for HardReader<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(HardReader::read(self, buf).await)
    }
}
