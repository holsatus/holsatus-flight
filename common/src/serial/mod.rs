use embedded_io::ReadExactError;
use grantable_io::{Reader, Writer};
use heapless::String;
use maitake_sync::{blocking, Mutex, MutexGuard};

pub mod params;
use params::fnv1a_hash_u32;

use embedded_io_async::BufRead;
use heapless::Vec;

use crate::{
    errors::adapter::embedded_io::EmbeddedIoError, utils::buf_read_ext::BufReadExt, MAX_IO_STREAMS,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamName(String<16>, StreamId);

impl StreamName {
    pub fn new(name: &str) -> Option<Self> {
        let name = String::try_from(name).ok()?;
        let id = StreamId(fnv1a_hash_u32(&name));
        Some(StreamName(name, id))
    }

    pub fn id(&self) -> StreamId {
        self.1
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(mav_param::Node, Default, Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StreamId(u32);

impl StreamId {
    pub const fn new(name: &str) -> StreamId {
        StreamId(fnv1a_hash_u32(&name))
    }
}

impl From<&str> for StreamId {
    fn from(value: &str) -> Self {
        StreamId(fnv1a_hash_u32(value))
    }
}

impl From<u32> for StreamId {
    fn from(value: u32) -> Self {
        StreamId(value)
    }
}

pub struct IoStreamList {
    list: blocking::Mutex<Vec<&'static IoStreamRaw<'static>, MAX_IO_STREAMS>>,
}

impl IoStreamList {
    pub const fn new() -> Self {
        IoStreamList {
            list: blocking::Mutex::new(Vec::new()),
        }
    }

    pub fn insert(&self, stream: &'static IoStreamRaw<'static>) -> Result<(), Error> {
        self.list.with_lock(|list| {
            if list.iter().all(|io_inner| *io_inner != stream) {
                list.push(stream).map_err(|_| Error::ListFull)
            } else {
                Err(Error::AlreadyInserted(stream.name.id()))
            }
        })
    }

    pub fn claim(&self, ident: impl Into<StreamId>) -> Result<IoStream, Error> {
        let stream_id = ident.into();
        self.list.with_lock(|list| {
            for raw in list.iter() {
                if raw.name.id() == stream_id {
                    return IoStream::claim_raw(*raw);
                }
            }
            Err(Error::DoesNotExist(stream_id))
        })
    }
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("Stream {0:?} already exists in the list")]
    AlreadyInserted(StreamId),
    #[error("Stream {0:?} has already been claimed")]
    AlreadyClaimed(StreamId),
    #[error("Stream {0:?} does not exist in the list")]
    DoesNotExist(StreamId),
    #[error("The list is already full")]
    ListFull,
}

static GLOBAL_STREAMS: IoStreamList = IoStreamList::new();

pub fn insert(stream: &'static IoStreamRaw<'static>) -> Result<(), Error> {
    GLOBAL_STREAMS.insert(stream)
}

pub fn claim(ident: impl Into<StreamId>) -> Result<IoStream, Error> {
    GLOBAL_STREAMS.claim(ident)
}

/// Statically allocated soft-half of the IO device.
///
/// Note, while the reader and writer are locked using separate mutexes to
/// allow for using their MutexGuards separately, they are normally locked
/// and unlocked together.
pub struct IoStreamRaw<'a> {
    name: StreamName,
    reader: Mutex<Reader<'a, EmbeddedIoError>>,
    writer: Mutex<Writer<'a, EmbeddedIoError>>,
}

impl PartialEq for &IoStreamRaw<'_> {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self, other)
    }
}

impl<'a> IoStreamRaw<'a> {
    pub fn new(
        name: &str,
        reader: Reader<'a, EmbeddedIoError>,
        writer: Writer<'a, EmbeddedIoError>,
    ) -> Self {
        IoStreamRaw {
            name: StreamName::new(name).unwrap(),
            reader: Mutex::new(reader),
            writer: Mutex::new(writer),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IoStream {
    pub reader: IoReader,
    pub writer: IoWriter,
}

impl IoStream {
    pub fn split(self) -> (IoReader, IoWriter) {
        (self.reader, self.writer)
    }

    pub fn name(&self) -> &StreamName {
        &self.reader.raw.name
    }

    pub fn claim_raw(raw: &'static IoStreamRaw<'static>) -> Result<Self, Error> {
        Ok(IoStream {
            reader: IoReader::claim_raw(raw)?,
            writer: IoWriter::claim_raw(raw)?,
        })
    }
}

pub struct IoReader {
    reader: MutexGuard<'static, Reader<'static, EmbeddedIoError>>,
    raw: &'static IoStreamRaw<'static>,
}

impl core::fmt::Debug for IoReader {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "IoReader: addr {}",
            core::ptr::addr_of!(self.raw) as usize
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for IoReader {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "IoReader: addr {}",
            core::ptr::addr_of!(self.raw) as usize
        )
    }
}

impl PartialEq for IoReader {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(&self.raw, &other.raw)
    }
}

impl Drop for IoReader {
    fn drop(&mut self) {
        let name = self.name().as_str();
        trace!("Dropping IoReader for: {}", name);
    }
}

impl IoReader {
    pub fn claim_raw(raw: &'static IoStreamRaw<'static>) -> Result<Self, Error> {
        Ok(IoReader {
            reader: raw
                .reader
                .try_lock()
                .ok_or(Error::AlreadyClaimed(raw.name.id()))?,
            raw,
        })
    }

    pub fn name(&self) -> &StreamName {
        &self.raw.name
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EmbeddedIoError> {
        self.reader.read(buf).await
    }

    pub async fn fill_buf(&mut self) -> Result<&[u8], EmbeddedIoError> {
        self.reader.fill_buf().await
    }

    pub fn consume(&mut self, amt: usize) {
        self.reader.consume(amt);
    }

    pub async fn skip_until(
        &mut self,
        delim: u8,
    ) -> Result<usize, ReadExactError<EmbeddedIoError>> {
        BufReadExt::skip_until(self, delim).await
    }
}

pub struct IoWriter {
    writer: MutexGuard<'static, Writer<'static, EmbeddedIoError>>,
    raw: &'static IoStreamRaw<'static>,
}

impl core::fmt::Debug for IoWriter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::write!(
            f,
            "IoWriter: addr {}",
            core::ptr::addr_of!(self.raw) as usize
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for IoWriter {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "IoWriter: addr {}",
            core::ptr::addr_of!(self.raw) as usize
        )
    }
}

impl PartialEq for IoWriter {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(&self.raw, &other.raw)
    }
}

impl Drop for IoWriter {
    fn drop(&mut self) {
        let name = self.name().as_str();
        trace!("Dropping IoWriter for: {}", name);
    }
}

impl IoWriter {
    pub fn name(&self) -> &StreamName {
        &self.raw.name
    }

    pub fn claim_raw(raw: &'static IoStreamRaw<'static>) -> Result<Self, Error> {
        Ok(IoWriter {
            writer: raw
                .writer
                .try_lock()
                .ok_or(Error::AlreadyClaimed(raw.name.id()))?,
            raw,
        })
    }
}

impl embedded_io_async::ErrorType for IoReader {
    type Error = EmbeddedIoError;
}

impl embedded_io_async::Read for IoReader {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.reader.read(buf).await
    }
}

impl embedded_io_async::BufRead for IoReader {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        self.reader.fill_buf().await
    }

    fn consume(&mut self, amt: usize) {
        self.reader.consume(amt)
    }
}

impl embedded_io_async::ErrorType for IoWriter {
    type Error = EmbeddedIoError;
}

impl embedded_io_async::Write for IoWriter {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.writer.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.writer.flush().await
    }
}

impl embedded_io_async::ErrorType for IoStream {
    type Error = EmbeddedIoError;
}

impl embedded_io_async::Read for IoStream {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.reader.read(buf).await
    }
}

impl embedded_io_async::Write for IoStream {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.writer.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.writer.flush().await
    }
}
