use embedded_io;
use embedded_io_061;

use embedded_io_async;
use embedded_io_async_061;
use futures::TryFutureExt;

pub struct CompatSyncWriter<W>(pub W);
pub struct CompatAsyncWriter<W>(pub W);

#[derive(Debug)]
pub struct Error<E>(pub E);

impl <E: embedded_io::Error> embedded_io_061::Error for Error<E> {
    fn kind(&self) -> embedded_io_061::ErrorKind {
        match self.0.kind() {
            embedded_io::ErrorKind::Other => embedded_io_061::ErrorKind::Other,
            embedded_io::ErrorKind::NotFound => embedded_io_061::ErrorKind::NotFound,
            embedded_io::ErrorKind::PermissionDenied => embedded_io_061::ErrorKind::PermissionDenied,
            embedded_io::ErrorKind::ConnectionRefused => embedded_io_061::ErrorKind::ConnectionRefused,
            embedded_io::ErrorKind::ConnectionReset => embedded_io_061::ErrorKind::ConnectionReset,
            embedded_io::ErrorKind::ConnectionAborted => embedded_io_061::ErrorKind::ConnectionAborted,
            embedded_io::ErrorKind::NotConnected => embedded_io_061::ErrorKind::NotConnected,
            embedded_io::ErrorKind::AddrInUse => embedded_io_061::ErrorKind::AddrInUse,
            embedded_io::ErrorKind::AddrNotAvailable => embedded_io_061::ErrorKind::AddrNotAvailable,
            embedded_io::ErrorKind::BrokenPipe => embedded_io_061::ErrorKind::BrokenPipe,
            embedded_io::ErrorKind::AlreadyExists => embedded_io_061::ErrorKind::AlreadyExists,
            embedded_io::ErrorKind::InvalidInput => embedded_io_061::ErrorKind::InvalidInput,
            embedded_io::ErrorKind::InvalidData => embedded_io_061::ErrorKind::InvalidData,
            embedded_io::ErrorKind::TimedOut => embedded_io_061::ErrorKind::TimedOut,
            embedded_io::ErrorKind::Interrupted => embedded_io_061::ErrorKind::Interrupted,
            embedded_io::ErrorKind::Unsupported => embedded_io_061::ErrorKind::Unsupported,
            embedded_io::ErrorKind::OutOfMemory => embedded_io_061::ErrorKind::OutOfMemory,
            embedded_io::ErrorKind::WriteZero => embedded_io_061::ErrorKind::WriteZero,
            _ => todo!(),
        }
    }
}

impl <W: embedded_io::Write> embedded_io_061::ErrorType for CompatSyncWriter<W> {
    type Error = Error<W::Error>;
}

impl <W: embedded_io::Write> embedded_io_061::Write for CompatSyncWriter<W> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).map_err(Error)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush().map_err(Error)
    }
}

impl <W: embedded_io_async::Write> embedded_io_async_061::ErrorType for CompatAsyncWriter<W> {
    type Error = Error<W::Error>;
}

impl <W: embedded_io_async::Write> embedded_io_async_061::Write for CompatAsyncWriter<W> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).map_err(Error).await
    }
}