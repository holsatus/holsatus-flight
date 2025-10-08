use super::bbbuffer::{GrantR, GrantW};

use super::SerialState;

/// Thin wrapper around a [`bbqueue::GrantR`] that also holds a reference
/// to a waker to alert the writer-half of the bytes consumed.
pub struct GrantWriter<'a> {
    pub(crate) state: &'a SerialState,
    pub(crate) inner: GrantW<'a>,
}

impl GrantWriter<'_> {
    /// Copy the largest possible amount of bytes from the grant
    /// to the given buffer. Whichever is shorter decides the number
    /// of bytes written. The return value is the amount copied.
    pub fn copy_max_from(&mut self, buf: &[u8]) -> usize {
        // Maximum number of bytes that can be copied contiguously
        let amount = self.inner.buf().len().min(buf.len());

        // Copy `amount` bytes from `grant` to `buf`
        self.inner.buf()[..amount].copy_from_slice(&buf[..amount]);

        // Release the copied amount on drop
        self.inner.to_commit(amount);

        self.state.wait_reader.wake();

        // The number copied
        amount
    }

    pub fn commit(self, bytes: usize) {
        self.inner.commit(bytes);
        self.state.wait_reader.wake();
    }

    pub fn buffer_mut(&mut self) -> &mut [u8] {
        self.inner.buf()
    }
}

/// Thin wrapper around a [`bbqueue::GrantR`] that also holds a reference
/// to a waker to alert the writer-half of the bytes consumed.
pub struct GrantReader<'a> {
    pub(crate) state: &'a SerialState,
    pub(crate) inner: GrantR<'a>,
}

impl GrantReader<'_> {
    /// Copy the largest possible amount of bytes from the grant
    /// to the given buffer. Whichever is shorter decides the number
    /// of bytes written. The return value is the amount copied.
    pub fn copy_max_into(&mut self, buf: &mut [u8]) -> usize {
        // Maximum number of bytes that can be copied contiguously
        let amount = self.inner.buf().len().min(buf.len());

        // Copy `amount` bytes from `grant` to `buf`
        buf[..amount].copy_from_slice(&self.inner.buf()[..amount]);

        // Release the copied amount on drop
        self.inner.to_release(amount);
        self.state.wait_writer.wake();

        // The number copied
        amount
    }

    pub fn release(self, bytes: usize) {
        self.inner.release(bytes);
        self.state.wait_writer.wake();
    }

    pub fn buffer(&self) -> &[u8] {
        self.inner.buf()
    }

    pub fn buffer_mut(&mut self) -> &mut [u8] {
        self.inner.buf_mut()
    }
}
