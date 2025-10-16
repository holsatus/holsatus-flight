use core::future::Future;

use embedded_io_async::{Read, Write};

use crate::errors::adapter::embedded_io::EmbeddedIoError;

pub(super) mod arming;
pub(super) mod calibrate;

#[cfg(feature = "mavlink")]
pub(super) mod mavlink;
pub(super) mod system;

pub(super) mod inspect;
pub(super) mod params;

pub trait CommandHandler {
    fn handler(
        &self,
        serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> impl Future<Output = Result<(), EmbeddedIoError>>;
}
