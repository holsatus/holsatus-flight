/// A `BytesReader` is a simple helper type to read bytes or subslices from a slice.
/// This is most useful for parsers, where you might want to read sequential bytes
/// in stages, as the parsers state machine progesses.
pub(crate) struct BytesReader<'a> {
    buf: &'a [u8],
    idx: usize,
}

impl<'a> BytesReader<'a> {
    /// Construct a new `BytesReader`
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, idx: 0 }
    }

    /// Get a slice of the remaining part of the buffer
    pub fn remaining(&self) -> &'a [u8] {
        debug_assert!(self.idx <= self.buf.len());
        self.buf.get(self.idx..).unwrap_or(&[])
    }

    /// Returns true if the buffer is empty
    pub fn is_empty(&self) -> bool {
        self.idx == self.buf.len()
    }

    /// Get the next byte from the buffer, if any
    pub fn next(&mut self) -> Option<u8> {
        if let Some(val) = self.buf.get(self.idx) {
            self.idx += 1;
            Some(*val)
        } else {
            None
        }
    }

    /// Returns up to the next `n` bytes from the buffer, or the
    /// remaining bytes if there are less than `n` bytes left.
    pub fn next_n(&mut self, n: usize) -> &[u8] {
        let end_idx = (self.idx + n).min(self.buf.len());
        let data = &self.buf[self.idx..end_idx];
        self.idx = end_idx;
        data
    }
}

#[cfg(test)]
mod tests {
    use super::BytesReader;

    #[test]
    fn test_bytes_reader() {
        let bytes: &[u8] = &[1, 2, 3, 4, 5];
        let mut reader = BytesReader::new(bytes);
        assert_eq!(reader.next(), Some(1));
        assert_eq!(reader.remaining(), &[2, 3, 4, 5]);
        assert_eq!(reader.next_n(2), &[2, 3]);
        assert_eq!(reader.remaining(), &[4, 5]);
        assert_eq!(reader.next(), Some(4));
        assert_eq!(reader.is_empty(), false);
        assert_eq!(reader.next(), Some(5));
        assert_eq!(reader.remaining(), &[]);
        assert_eq!(reader.next(), None);
        assert_eq!(reader.is_empty(), true);
    }
}
