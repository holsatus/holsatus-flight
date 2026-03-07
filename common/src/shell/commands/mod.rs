use core::future::Future;

use embassy_time::{Duration, Ticker};
use embedded_io_async::{Read, Write};

use crate::errors::adapter::embedded_io::EmbeddedIoError;

pub(super) mod arming;
pub(super) mod calibrate;

#[cfg(feature = "mavlink")]
pub(super) mod mavlink;
pub(super) mod system;

pub(super) mod inspect;
pub(super) mod motor;
pub(super) mod params;

pub trait CommandHandler {
    fn handler(
        &self,
        serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> impl Future<Output = Result<(), EmbeddedIoError>>;
}

const MAX_STREAM_HZ: u8 = 10;

pub(super) async fn periodic_ticker(
    serial: &mut impl Write<Error = EmbeddedIoError>,
    frequency: Option<u8>,
) -> Result<Option<Ticker>, EmbeddedIoError> {
    let Some(requested_hz) = frequency else {
        return Ok(None);
    };

    if requested_hz == 0 {
        serial
            .write_all(b"[warn] frequency must be >= 1 Hz; showing one snapshot\n\r")
            .await?;
        return Ok(None);
    }

    let effective_hz = requested_hz.min(MAX_STREAM_HZ);
    if effective_hz != requested_hz {
        serial
            .write_all(b"[warn] frequency limited to 10 Hz for system stability\n\r")
            .await?;
    }

    Ok(Some(Ticker::every(Duration::from_hz(effective_hz as u64))))
}

/// Returns `true` when the caller should exit the periodic display loop.
pub(super) async fn wait_next_or_ctrl_c(
    _serial: &mut impl embedded_io_async::Read<Error = EmbeddedIoError>,
    maybe_ticker: &mut Option<Ticker>,
) -> Result<bool, EmbeddedIoError> {
    let Some(ticker) = maybe_ticker else {
        return Ok(true);
    };

    // Important: Avoid canceling in-flight serial reads from `select` loops.
    // Some buffered stream implementations are not cancellation-safe and can
    // deadlock if a read future is dropped after partially acquiring state.
    ticker.next().await;
    Ok(false)
}
