use maitake_sync::WaitCell;

mod error;
use error::AtomicError;

mod grant;
pub use grant::{GrantReader, GrantWriter};

mod hard;
pub use hard::{HardReader, HardWriter};

mod soft;
pub use soft::{SoftReader, SoftWriter, BufReadExt};

mod bbbuffer;
pub use bbbuffer::BufferState;


/// Internal state to back a [`SerialWriter`] or [`SerialReader`] connection
///
/// The usage requires a unique statically allocated instance [`SerialState`]
/// instance.
///
/// # Example
/// ```rust
/// use holsatus_sync::serial::{SerialState, SerialWriter, StaticBuf};
///
/// static STATE: SerialState = SerialState::new();
/// static BUFFER: StaticBuf<128> = StaticBuf::new();
/// let serial = SerialWriter::new(&STATE, BUFFER.take()).unwrap();
/// ```
///
/// Alternatively, consider using the macro
///
/// ```rust
/// let serial = holsatus_sync::new_serial_writer!(128);
///
/// // split into reading and writing halves
/// let (writer, reader) = serial.split();
/// ```
pub struct SerialState {
    buffer: BufferState,
    wait_writer: WaitCell,
    wait_reader: WaitCell,
    error: AtomicError,
}

impl SerialState {
    /// Creates a new [`SerialState`] instance.
    ///
    /// This initializes the internal buffer and synchronization primitives required
    /// for serial communication.
    ///
    /// # Example
    ///
    /// ```rust
    /// use holsatus_sync::serial::SerialState;
    ///
    /// static STATE: SerialState = SerialState::new();
    /// ```
    pub const fn new() -> Self {
        Self {
            buffer: BufferState::new(),
            wait_writer: WaitCell::new(),
            wait_reader: WaitCell::new(),
            error: AtomicError::new(),
        }
    }
}

impl Default for SerialState {
    fn default() -> Self {
        SerialState::new()
    }
}

/// A [`SerialWriter`] is used to buffer an async TX serial connection.
///
/// This struct provides access to both the hardware-facing and software-facing
/// ends of the serial connection. It allows splitting into separate hardware
/// and software writers.
///
/// # Example
///
/// ```rust
/// use holsatus_sync::serial::{SerialState, SerialWriter, StaticBuf};
///
/// static STATE: SerialState = SerialState::new();
/// static BUFFER: StaticBuf<128> = StaticBuf::new();
/// let serial = SerialWriter::new(&STATE, BUFFER.take()).unwrap();
///
/// // Split into hardware and software writers
/// let (hardware, software) = serial.split();
/// ```
pub struct SerialWriter<'a> {
    pub software: soft::SoftWriter<'a>,
    pub hardware: hard::HardReader<'a>,
}

impl<'a> SerialWriter<'a> {
    /// Creates a new [`SerialWriter`] instance.
    ///
    /// This initializes the hardware and software writers using the provided
    /// [`SerialState`].
    ///
    /// # Parameters
    ///
    /// - `state`: A reference to a statically allocated [`SerialState`] instance.
    ///
    /// # Returns
    ///
    /// - `Some(SerialWriter)` if the buffer could be successfully split.
    /// - `None` if the buffer could not be split.
    ///
    /// # Example
    ///
    /// ```rust
    /// use holsatus_sync::serial::{SerialState, SerialWriter, StaticBuf};
    ///
    /// static STATE: SerialState = SerialState::new();
    /// static BUFFER: StaticBuf<128> = StaticBuf::new();
    /// let serial = SerialWriter::new(&STATE, BUFFER.take()).unwrap();
    /// ```
    pub fn new(state: &'a SerialState, buf: &'a mut [u8]) -> Option<SerialWriter<'a>> {
        let (producer, consumer) = state.buffer.init(buf)?;

        let software = soft::SoftWriter {
            producer,
            state,
        };

        let hardware = hard::HardReader {
            consumer,
            state,
        };

        Some(SerialWriter { software, hardware })
    }

    /// Splits the [`SerialWriter`] into its hardware and software components.
    ///
    /// This allows separate access to the hardware-facing and software-facing
    /// ends of the serial connection.
    ///
    /// # Returns
    ///
    /// A tuple containing:
    /// - `SoftWriter`: The software-facing writer.
    /// - `HardReader`: The hardware-facing reader.
    ///
    /// # Example
    ///
    /// ```rust
    /// use holsatus_sync::serial::{SerialState, SerialWriter, StaticBuf};
    ///
    /// static STATE: SerialState = SerialState::new();
    /// static BUFFER: StaticBuf<128> = StaticBuf::new();
    /// let serial = SerialWriter::new(&STATE, BUFFER.take()).unwrap();
    ///
    /// let (hardware, software) = serial.split();
    /// ```
    pub fn split(self) -> (soft::SoftWriter<'a>, hard::HardReader<'a>) {
        let SerialWriter {
            software: hardware,
            hardware: software,
        } = self;
        (hardware, software)
    }
}

/// Creates a new [`SerialWriter`] instance with a statically allocated [`SerialState`].
///
/// This macro simplifies the creation of a [`SerialWriter`] by automatically
/// allocating the required [`SerialState`] instance.
///
/// # Parameters
///
/// - `$n`: The size of the buffer.
///
/// # Example
///
/// ```rust
/// let serial = holsatus_sync::new_serial_writer!(128);
///
/// // Split into hardware and software writers
/// let (hardware, software) = serial.split();
/// ```
#[macro_export]
macro_rules! new_serial_writer {
    ($n:literal) => {{
        use $crate::sync::serial::{SerialState, StaticBuf, SerialWriter};
        static STATE: SerialState = SerialState::new();
        static BUFFER: StaticBuf<$n> = StaticBuf::new();
        SerialWriter::new(&STATE, BUFFER.take()).unwrap()
    }};
}

/// A [`SerialReader`] is used to buffer an async RX serial connection.
///
/// This struct provides access to both the hardware-facing and software-facing
/// ends of the serial connection. It allows splitting into separate hardware
/// and software readers.
///
/// # Example
///
/// ```rust
/// use holsatus_sync::serial::{SerialState, SerialReader, StaticBuf};
///
/// static STATE: SerialState = SerialState::new();
/// static BUFFER: StaticBuf<128> = StaticBuf::new();
/// let serial = SerialReader::new(&STATE, BUFFER.take()).unwrap();
///
/// // Split into hardware and software readers
/// let (hardware, software) = serial.split();
/// ```
pub struct SerialReader<'a> {
    pub hardware: hard::HardWriter<'a>,
    pub software: soft::SoftReader<'a>,
}

impl<'a> SerialReader<'a> {
    /// Creates a new [`SerialReader`] instance.
    ///
    /// This initializes the hardware and software readers using the provided
    /// [`SerialState`].
    ///
    /// # Parameters
    ///
    /// - `state`: A reference to a statically allocated [`SerialState`] instance.
    ///
    /// # Returns
    ///
    /// - `Some(SerialReader)` if the buffer could be successfully split.
    /// - `None` if the buffer could not be split.
    ///
    /// # Example
    ///
    /// ```rust
    /// use holsatus_sync::serial::{SerialState, SerialReader, StaticBuf};
    ///
    /// static STATE: SerialState = SerialState::new();
    /// static BUFFER: StaticBuf<128> = StaticBuf::new();
    /// let serial = SerialReader::new(&STATE, BUFFER.take()).unwrap();
    /// ```
    pub fn new(state: &'a SerialState, buf: &'a mut [u8]) -> Option<SerialReader<'a>> {
        let (producer, consumer) = state.buffer.init(buf)?;

        let hardware = hard::HardWriter {
            producer,
            state,
        };

        let software = soft::SoftReader {
            consumer,
            state,
            grant: None
        };

        Some(SerialReader { hardware, software })
    }

    /// Splits the [`SerialReader`] into its hardware and software components.
    ///
    /// This allows separate access to the hardware-facing and software-facing
    /// ends of the serial connection.
    ///
    /// # Returns
    ///
    /// A tuple containing:
    /// - `HardWriter`: The hardware-facing writer.
    /// - `SoftReader`: The software-facing reader.
    ///
    /// # Example
    ///
    /// ```rust
    /// use holsatus_sync::serial::{SerialState, SerialReader, StaticBuf};
    ///
    /// static STATE: SerialState = SerialState::new();
    /// static BUFFER: StaticBuf<128> = StaticBuf::new();
    /// let serial = SerialReader::new(&STATE, BUFFER.take()).unwrap();
    ///
    /// let (hardware, software) = serial.split();
    /// ```
    pub fn split(self) -> (hard::HardWriter<'a>, soft::SoftReader<'a>) {
        let SerialReader { hardware, software } = self;
        (hardware, software)
    }
}

/// Creates a new [`SerialReader`] instance with a statically allocated [`SerialState`].
///
/// This macro simplifies the creation of a [`SerialReader`] by automatically
/// allocating the required [`SerialState`] instance.
///
/// # Parameters
///
/// - `$n`: The size of the buffer.
///
/// # Example
///
/// ```rust
/// let serial = holsatus_sync::new_serial_reader!(128);
///
/// // Split into hardware and software readers
/// let (hardware, software) = serial.split();
/// ```
#[macro_export]
macro_rules! new_serial_reader {
    ($n:literal) => {{
        use $crate::sync::serial::{SerialState, StaticBuf, SerialReader};
        static STATE: SerialState = SerialState::new();
        static BUFFER: StaticBuf<$n> = StaticBuf::new();
        SerialReader::new(&STATE, BUFFER.take()).unwrap()
    }};
}

/// Used for statically allocating a buffer that requires a mutable reference
pub struct StaticBuf<const N: usize> {
    buf: static_cell::ConstStaticCell<[u8; N]>
}

impl<const N: usize> Default for StaticBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl <const N: usize> StaticBuf<N> {
    pub const fn new() -> Self {
        Self {
            buf: static_cell::ConstStaticCell::new([0u8; N])
        }
    }

    pub fn take(&'static self) -> &'static mut [u8; N] {
        self.buf.take()
    }


    pub fn try_take(&'static self) -> Option<&'static mut [u8; N]> {
        self.buf.try_take()
    }
}