use core::{
    cmp::min,
    marker::PhantomData,
    mem::forget,
    ptr::NonNull,
    slice::{from_raw_parts, from_raw_parts_mut},
};

use portable_atomic::{
    AtomicBool, AtomicUsize,
    Ordering::{AcqRel, Acquire, Release},
};

/// Result type used by the `BBQueue` interfaces
pub type Result<T> = core::result::Result<T, Error>;

/// Error type used by the `BBQueue` interfaces
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Error {
    /// The buffer does not contain sufficient size for the requested action
    InsufficientSize,

    /// Unable to produce another grant, a grant of this type is already in
    /// progress
    GrantInProgress,
}

#[derive(Debug)]
/// A backing structure for a BBQueue. Can be used to create either
/// a BBQueue or a split Producer/Consumer pair
pub struct BufferState {
    initialized: AtomicBool,

    /// Where the next byte will be written
    write: AtomicUsize,

    /// Where the next byte will be read from
    read: AtomicUsize,

    /// Used in the inverted case to mark the end of the
    /// readable streak. Otherwise will == sizeof::<self.buf>().
    /// Writer is responsible for placing this at the correct
    /// place when entering an inverted condition, and Reader
    /// is responsible for moving it back to sizeof::<self.buf>()
    /// when exiting the inverted condition
    last: AtomicUsize,

    /// Used by the Writer to remember what bytes are currently
    /// allowed to be written to, but are not yet ready to be
    /// read from
    reserve: AtomicUsize,

    /// Is there an active read grant?
    read_in_progress: AtomicBool,

    /// Is there an active write grant?
    write_in_progress: AtomicBool,
}

impl Default for BufferState {
    fn default() -> Self {
        Self::new()
    }
}

impl BufferState {
    /// Create a new constant inner portion of a `BBBuffer`.
    ///
    /// NOTE: This is only necessary to use when creating a `BBBuffer` at static
    /// scope, and is generally never used directly. This process is necessary to
    /// work around current limitations in `const fn`, and will be replaced in
    /// the future.
    pub const fn new() -> Self {
        Self {
            initialized: AtomicBool::new(false),

            // Owned by the writer
            write: AtomicUsize::new(0),

            // Owned by the reader
            read: AtomicUsize::new(0),

            // Cooperatively owned
            //
            // NOTE: This should generally be initialized as size_of::<self.buf>(), however
            // this would prevent the structure from being entirely zero-initialized,
            // and can cause the .data section to be much larger than necessary. By
            // forcing the `last` pointer to be zero initially, we place the structure
            // in an "inverted" condition, which will be resolved on the first commited
            // bytes that are written to the structure.
            //
            // When read == last == write, no bytes will be allowed to be read (good), but
            // write grants can be given out (also good).
            last: AtomicUsize::new(0),

            // Owned by the Writer, "private"
            reserve: AtomicUsize::new(0),

            // Owned by the Reader, "private"
            read_in_progress: AtomicBool::new(false),

            // Owned by the Writer, "private"
            write_in_progress: AtomicBool::new(false),
            // We haven't split at the start
        }
    }

    /// Attempt to split the `BBBuffer` into `Consumer` and `Producer` halves to gain access to the
    /// buffer. If buffer has already been split, an error will be returned.
    ///
    /// NOTE: When splitting, the underlying buffer will be explicitly initialized
    /// to zero. This may take a measurable amount of time, depending on the size
    /// of the buffer. This is necessary to prevent undefined behavior. If the buffer
    /// is placed at `static` scope within the `.bss` region, the explicit initialization
    /// will be elided (as it is already performed as part of memory initialization)
    ///
    /// NOTE:  If the `thumbv6` feature is selected, this function takes a short critical section
    /// while splitting.
    pub fn init(&self, buf: &mut [u8]) -> Option<(Producer<'_>, Consumer<'_>)> {
        if self.initialized.swap(true, AcqRel) {
            return None;
        }

        Some((
            Producer {
                buf: NonNull::from(&*buf),
                state: NonNull::from(self),
                pd: PhantomData,
            },
            Consumer {
                buf: NonNull::from(&*buf),
                state: NonNull::from(self),
                pd: PhantomData,
            },
        ))
    }
}

/// `Producer` is the primary interface for pushing data into a `BBBuffer`.
/// There are various methods for obtaining a grant to write to the buffer, with
/// different potential tradeoffs. As all grants are required to be a contiguous
/// range of data, different strategies are sometimes useful when making the decision
/// between maximizing usage of the buffer, and ensuring a given grant is successful.
///
/// As a short summary of currently possible grants:
///
/// * `grant_exact(N)`
///   * User will receive a grant `sz == N` (or receive an error)
///   * This may cause a wraparound if a grant of size N is not available
///     at the end of the ring.
///   * If this grant caused a wraparound, the bytes that were "skipped" at the
///     end of the ring will not be available until the reader reaches them,
///     regardless of whether the grant commited any data or not.
///   * Maximum possible waste due to skipping: `N - 1` bytes
/// * `grant_max_remaining(N)`
///   * User will receive a grant `0 < sz <= N` (or receive an error)
///   * This will only cause a wrap to the beginning of the ring if exactly
///     zero bytes are available at the end of the ring.
///   * Maximum possible waste due to skipping: 0 bytes
///
/// See [this github issue](https://github.com/jamesmunns/bbqueue/issues/38) for a
/// discussion of grant methods that could be added in the future.
#[derive(Debug)]
pub struct Producer<'a> {
    buf: NonNull<[u8]>,
    pub(crate) state: NonNull<BufferState>,
    pd: PhantomData<&'a ()>,
}

unsafe impl Send for Producer<'_> {}

impl<'a> Producer<'a> {
    /// Request a writable, contiguous section of memory of exactly
    /// `sz` bytes. If the buffer size requested is not available,
    /// an error will be returned.
    ///
    /// This method may cause the buffer to wrap around early if the
    /// requested space is not available at the end of the buffer, but
    /// is available at the beginning
    pub fn grant_exact(&mut self, sz: usize) -> Result<GrantW<'a>> {
        let inner = unsafe { &self.state.as_ref() };

        if inner.write_in_progress.swap(true, AcqRel) {
            return Err(Error::GrantInProgress);
        }

        // Writer component. Must never write to `read`,
        // be careful writing to `load`
        let write = inner.write.load(Acquire);
        let read = inner.read.load(Acquire);
        let already_inverted = write < read;

        let start = if already_inverted {
            if (write + sz) < read {
                // Inverted, room is still available
                write
            } else {
                // Inverted, no room is available
                inner.write_in_progress.store(false, Release);
                return Err(Error::InsufficientSize);
            }
        } else {
            #[allow(clippy::collapsible_if)]
            if write + sz <= self.buf.len() {
                // Non inverted condition
                write
            } else {
                // Not inverted, but need to go inverted

                // NOTE: We check sz < read, NOT <=, because
                // write must never == read in an inverted condition, since
                // we will then not be able to tell if we are inverted or not
                if sz < read {
                    // Invertible situation
                    0
                } else {
                    // Not invertible, no space
                    inner.write_in_progress.store(false, Release);
                    return Err(Error::InsufficientSize);
                }
            }
        };

        // Safe write, only viewed by this task
        inner.reserve.store(start + sz, Release);

        // This is sound, as UnsafeCell, MaybeUninit, and GenericArray
        // are all `#[repr(Transparent)]
        let grant_slice = unsafe { &self.buf.as_mut()[start..(start + sz)] };

        Ok(GrantW {
            buf: grant_slice.into(),
            bbq: self.state,
            phatom: PhantomData,
        })
    }

    /// Request a writable, contiguous section of memory of up to
    /// `sz` bytes. If a buffer of size `sz` is not available without
    /// wrapping, but some space (0 < available < sz) is available without
    /// wrapping, then a grant will be given for the remaining size at the
    /// end of the buffer. If no space is available for writing, an error
    /// will be returned.
    pub fn grant_max_remaining(&mut self) -> Result<GrantW<'a>> {
        let inner = unsafe { &self.state.as_ref() };

        if inner.write_in_progress.swap(true, AcqRel) {
            return Err(Error::GrantInProgress);
        }

        // Writer component. Must never write to `read`,
        // be careful writing to `load`
        let write = inner.write.load(Acquire);
        let read = inner.read.load(Acquire);
        let max = self.buf.len();
        let mut sz = max;

        let already_inverted = write < read;

        let start = if already_inverted {
            // In inverted case, read is always > write
            let remain = read - write - 1;

            if remain != 0 {
                sz = min(remain, sz);
                write
            } else {
                // Inverted, no room is available
                inner.write_in_progress.store(false, Release);
                return Err(Error::InsufficientSize);
            }
        } else {
            #[allow(clippy::collapsible_if)]
            if write != max {
                // Some (or all) room remaining in un-inverted case
                sz = min(max - write, sz);
                write
            } else {
                // Not inverted, but need to go inverted

                // NOTE: We check read > 1, NOT read >= 1, because
                // write must never == read in an inverted condition, since
                // we will then not be able to tell if we are inverted or not
                if read > 1 {
                    sz = min(read - 1, sz);
                    0
                } else {
                    // Not invertible, no space
                    inner.write_in_progress.store(false, Release);
                    return Err(Error::InsufficientSize);
                }
            }
        };

        // Safe write, only viewed by this task
        inner.reserve.store(start + sz, Release);

        // This is sound, as UnsafeCell, MaybeUninit, and GenericArray
        // are all `#[repr(Transparent)]
        let grant_slice = unsafe { &self.buf.as_mut()[start..(start + sz)] };

        Ok(GrantW {
            buf: grant_slice.into(),
            bbq: self.state,
            phatom: PhantomData,
        })
    }
}

/// `Consumer` is the primary interface for reading data from a `BBBuffer`.
#[derive(Debug)]
pub struct Consumer<'a> {
    buf: NonNull<[u8]>,
    pub(crate) state: NonNull<BufferState>,
    pd: PhantomData<&'a ()>,
}

unsafe impl Send for Consumer<'_> {}

impl<'a> Consumer<'a> {
    /// Obtains a contiguous slice of committed bytes. This slice may not
    /// contain ALL available bytes, if the writer has wrapped around. The
    /// remaining bytes will be available after all readable bytes are
    /// released
    pub fn read(&mut self) -> Result<GrantR<'a>> {
        let inner = unsafe { &self.state.as_ref() };

        if inner.read_in_progress.swap(true, AcqRel) {
            return Err(Error::GrantInProgress);
        }

        let write = inner.write.load(Acquire);
        let last = inner.last.load(Acquire);
        let mut read = inner.read.load(Acquire);

        // Resolve the inverted case or end of read
        if (read == last) && (write < read) {
            read = 0;
            // This has some room for error, the other thread reads this
            // Impact to Grant:
            //   Grant checks if read < write to see if inverted. If not inverted, but
            //     no space left, Grant will initiate an inversion, but will not trigger it
            // Impact to Commit:
            //   Commit does not check read, but if Grant has started an inversion,
            //   grant could move Last to the prior write position
            // MOVING READ BACKWARDS!
            inner.read.store(0, Release);
        }

        let sz = if write < read {
            // Inverted, only believe last
            last
        } else {
            // Not inverted, only believe write
            write
        } - read;

        if sz == 0 {
            inner.read_in_progress.store(false, Release);
            return Err(Error::InsufficientSize);
        }

        // This is sound, as UnsafeCell, MaybeUninit, and GenericArray
        // are all `#[repr(Transparent)]
        let grant_slice = unsafe { &self.buf.as_mut()[read..(read + sz)] };

        Ok(GrantR {
            buf: grant_slice.into(),
            bbq: self.state,
            phatom: PhantomData,
        })
    }
}

/// A structure representing a contiguous region of memory that
/// may be written to, and potentially "committed" to the queue.
///
/// NOTE: If the grant is dropped without explicitly commiting
/// the contents, or by setting a the number of bytes to
/// automatically be committed with `to_commit()`, then no bytes
/// will be comitted for writing.
///
/// If the `thumbv6` feature is selected, dropping the grant
/// without committing it takes a short critical section,
#[derive(Debug, PartialEq)]
pub struct GrantW<'a> {
    pub(crate) buf: NonNull<[u8]>,
    bbq: NonNull<BufferState>,
    phatom: PhantomData<&'a mut [u8]>,
}

unsafe impl Send for GrantW<'_> {}

/// A structure representing a contiguous region of memory that
/// may be read from, and potentially "released" (or cleared)
/// from the queue
///
/// NOTE: If the grant is dropped without explicitly releasing
/// the contents, or by setting the number of bytes to automatically
/// be released with `to_release()`, then no bytes will be released
/// as read.
///
///
/// If the `thumbv6` feature is selected, dropping the grant
/// without releasing it takes a short critical section,
#[derive(Debug, PartialEq)]
pub struct GrantR<'a> {
    pub(crate) buf: NonNull<[u8]>,
    bbq: NonNull<BufferState>,
    phatom: PhantomData<&'a mut [u8]>,
}

unsafe impl Send for GrantR<'_> {}

impl GrantW<'_> {
    /// Finalizes a writable grant given by `grant()` or `grant_max()`.
    /// This makes the data available to be read via `read()`. This consumes
    /// the grant.
    ///
    /// If `used` is larger than the given grant, the maximum amount will
    /// be commited
    ///
    /// NOTE:  If the `thumbv6` feature is selected, this function takes a short critical
    /// section while committing.
    pub fn commit(mut self, used: usize) {
        self.commit_inner(used);
        forget(self);
    }

    /// Obtain access to the inner buffer for writing
    pub fn buf_mut(&mut self) -> &mut [u8] {
        unsafe { from_raw_parts_mut(self.buf.as_ptr() as *mut u8, self.buf.len()) }
    }

    #[inline(always)]
    fn commit_inner(&mut self, used: usize) {
        let inner = unsafe { &self.bbq.as_ref() };

        // If there is no grant in progress, return early. This
        // generally means we are dropping the grant within a
        // wrapper structure
        if !inner.write_in_progress.load(Acquire) {
            return;
        }

        // Writer component. Must never write to READ,
        // be careful writing to LAST

        // Saturate the grant commit
        let len = self.buf.len();
        let used = min(len, used);

        let write = inner.write.load(Acquire);
        inner.reserve.fetch_sub(len - used, AcqRel);

        let max = self.buf.len();
        let last = inner.last.load(Acquire);
        let new_write = inner.reserve.load(Acquire);

        if (new_write < write) && (write != max) {
            // We have already wrapped, but we are skipping some bytes at the end of the ring.
            // Mark `last` where the write pointer used to be to hold the line here
            inner.last.store(write, Release);
        } else if new_write > last {
            // We're about to pass the last pointer, which was previously the artificial
            // end of the ring. Now that we've passed it, we can "unlock" the section
            // that was previously skipped.
            //
            // Since new_write is strictly larger than last, it is safe to move this as
            // the other thread will still be halted by the (about to be updated) write
            // value
            inner.last.store(max, Release);
        }
        // else: If new_write == last, either:
        // * last == max, so no need to write, OR
        // * If we write in the end chunk again, we'll update last to max next time
        // * If we write to the start chunk in a wrap, we'll update last when we
        //     move write backwards

        // Write must be updated AFTER last, otherwise read could think it was
        // time to invert early!
        inner.write.store(new_write, Release);

        // Allow subsequent grants
        inner.write_in_progress.store(false, Release);
    }
}

impl GrantR<'_> {
    /// Release a sequence of bytes from the buffer, allowing the space
    /// to be used by later writes. This consumes the grant.
    ///
    /// If `used` is larger than the given grant, the full grant will
    /// be released.
    ///
    /// NOTE:  If the `thumbv6` feature is selected, this function takes a short critical
    /// section while releasing.
    pub fn release(mut self, used: usize) {
        self.release_inner(used);
        forget(self);
    }

    /// Obtain access to the inner buffer for reading
    pub fn buf(&self) -> &[u8] {
        unsafe { from_raw_parts(self.buf.as_ptr() as *const u8, self.buf.len()) }
    }

    /// Obtain mutable access to the read grant
    ///
    /// This is useful if you are performing in-place operations
    /// on an incoming packet, such as decryption
    pub fn buf_mut(&mut self) -> &mut [u8] {
        unsafe { from_raw_parts_mut(self.buf.as_ptr() as *mut u8, self.buf.len()) }
    }

    #[inline(always)]
    fn release_inner(&mut self, used: usize) {
        let inner = unsafe { &self.bbq.as_ref() };

        // If there is no grant in progress, return early. This
        // generally means we are dropping the grant within a
        // wrapper structure
        if !inner.read_in_progress.load(Acquire) {
            return;
        }

        // Saturate the grant release
        let used = min(self.buf.len(), used);

        // This should always be checked by the public interfaces
        debug_assert!(used <= self.buf.len());

        // This should be fine, purely incrementing
        let _ = inner.read.fetch_add(used, Release);

        // Allow subsequent grants
        inner.read_in_progress.store(false, Release);
    }
}

impl Drop for GrantW<'_> {
    fn drop(&mut self) {
        self.commit_inner(0)
    }
}

impl Drop for GrantR<'_> {
    fn drop(&mut self) {
        self.release_inner(0)
    }
}
