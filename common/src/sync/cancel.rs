use core::future::Future;

use embassy_futures::select::{select, Either};
use maitake_sync::{blocking::DefaultMutex, WaitMap, WaitQueue};
use mutex::{ConstInit, ScopedRawMutex};

/// Allows one or more async operations to be efficiently canceled remotely
pub struct Cancel<M: ScopedRawMutex = DefaultMutex> {
    queue: WaitQueue<M>,
}

impl<M: ScopedRawMutex + ConstInit> Default for Cancel<M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: ScopedRawMutex> Cancel<M> {
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            queue: WaitQueue::new_with_raw_mutex(M::INIT),
        }
    }

    pub const fn get_token(&self) -> CancelToken<'_, M> {
        CancelToken { cancel: self }
    }

    pub fn cancel(&self) {
        self.queue.wake_all();
    }
}

pub struct CancelToken<'a, M: ScopedRawMutex = DefaultMutex> {
    cancel: &'a Cancel<M>,
}

impl<M: ScopedRawMutex> CancelToken<'_, M> {
    /// Run the provided future until completeion, or until the cancelation
    /// token is with [`Cancel::cancel`]. If the provided future runs to
    /// completion before being canceled, its value is returned.
    pub async fn run<F: Future>(&self, future: F) -> Option<F::Output> {
        let res = select(future, self.cancel.queue.wait()).await;

        if let Either::First(value) = res {
            Some(value)
        } else {
            None
        }
    }
}

/// Allows one or more async operations to be efficiently canceled remotely
pub struct IndexedCancel<I: PartialEq, M: ScopedRawMutex = DefaultMutex> {
    queue: WaitMap<I, (), M>,
}

impl<I: PartialEq, M: ScopedRawMutex + ConstInit> Default for IndexedCancel<I, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<I: PartialEq, M: ScopedRawMutex> IndexedCancel<I, M> {
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            queue: WaitMap::new_with_raw_mutex(M::INIT),
        }
    }

    pub const fn get_token(&self, index: I) -> IndexedCancelToken<'_, I, M> {
        IndexedCancelToken {
            cancel: self,
            index,
        }
    }

    pub fn cancel(&self, index: I) {
        self.queue.wake(&index, ());
    }
}

pub struct IndexedCancelToken<'a, I: PartialEq, M: ScopedRawMutex = DefaultMutex> {
    cancel: &'a IndexedCancel<I, M>,
    index: I,
}

impl<I: PartialEq + Clone, M: ScopedRawMutex> IndexedCancelToken<'_, I, M> {
    /// Run the provided future until completeion, or until the cancelation
    /// token is with [`Cancel::cancel`]. If the provided future runs to
    /// completion before being canceled, its value is returned.
    pub async fn run<F: Future>(&self, future: F) -> Option<F::Output> {
        let res = select(future, self.cancel.queue.wait(self.index.clone())).await;

        if let Either::First(value) = res {
            Some(value)
        } else {
            None
        }
    }
}
