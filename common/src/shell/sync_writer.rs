use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};

use embedded_io::{ErrorType, Write as SyncWrite};
use embedded_io_async::Write;

/// Wrapper to allow the CLI to own a synchronous writer. This is a bit of a
/// hack since embedded-cli does not support async (and takes ownership of the
/// writer..)
pub(super) struct SyncWriter<'a, W: Write<Error = E>, E: embedded_io::Error> {
    writer: &'a Mutex<NoopRawMutex, W>,
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> SyncWriter<'a, W, E> {
    pub fn new(writer: &'a Mutex<NoopRawMutex, W>) -> Self {
        Self { writer }
    }
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> ErrorType for SyncWriter<'a, W, E> {
    type Error = E;
}

impl<'a, W: Write<Error = E>, E: embedded_io::Error> SyncWrite for SyncWriter<'a, W, E> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // The mutex is expected to never be locked, since the CLI is evaluated
        // before anything else in the loop
        let mut writer = self.writer.try_lock().expect("Failed to lock writer");
        embassy_futures::block_on(writer.write(buf))
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // The mutex is expected to never be locked, since the CLI is evaluated
        // before anything else in the loop
        let mut writer = self.writer.try_lock().expect("Failed to lock writer");
        embassy_futures::block_on(writer.flush())
    }
}
