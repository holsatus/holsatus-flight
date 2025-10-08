use core::future::Future;

use embedded_io::ErrorKind;
use embedded_io_async::{Read, Write};

pub(super) mod arming;
pub(super) mod calibrate;

#[cfg(feature = "mavlink")]
pub(super) mod mavlink;
pub(super) mod system;

pub trait CommandHandler {
    fn handler(
        &self,
        serial: impl Read<Error = ErrorKind> + Write<Error = ErrorKind>,
    ) -> impl Future<Output = Result<(), ErrorKind>>;
}
