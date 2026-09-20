use core::{
    ops::{Deref, DerefMut},
    sync::atomic::Ordering,
};

use futures::FutureExt;
use maitake_sync::{RwLock, RwLockReadGuard, RwLockWriteGuard, WaitQueue};
use portable_atomic::AtomicU8;

/// No persisted values have been requested for this table.
const UNLOADED: u8 = 0;
/// A load is in progress; other callers should wait for it.
const LOADING: u8 = 1;
/// Persisted values have been loaded (or loading failed).
const LOADED: u8 = 2;

pub struct ParamTable<T: ?Sized> {
    name: &'static str,
    waiters: WaitQueue,
    generation: AtomicU8,
    load_state: AtomicU8,
    params: RwLock<T>,
}

pub struct TableReadGuard<'a, T: ?Sized> {
    guard: RwLockReadGuard<'a, T>,
}

pub struct TableWriteGuard<'a, T: ?Sized> {
    table: &'a ParamTable<T>,
    guard: RwLockWriteGuard<'a, T>,
}

impl<T: ?Sized> Deref for TableReadGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

impl<T: ?Sized> Deref for TableWriteGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.guard
    }
}

impl<T: ?Sized> DerefMut for TableWriteGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.guard
    }
}

impl<T: ?Sized> Drop for TableWriteGuard<'_, T> {
    fn drop(&mut self) {
        // Assume a write has updated the table in some way
        self.table.notify();
    }
}

impl<T: mav_param::Node> ParamTable<T> {
    pub const fn new(name: &'static str, data: T) -> Self {
        ParamTable {
            name,
            waiters: WaitQueue::new(),
            generation: AtomicU8::new(0),
            load_state: AtomicU8::new(UNLOADED),
            params: RwLock::new(data),
        }
    }

    pub const fn default(name: &'static str) -> Self
    where
        T: crate::utils::const_default::ConstDefault,
    {
        Self::new(name, T::DEFAULT)
    }

    pub async fn reset(&self)
    where
        T: crate::utils::const_default::ConstDefault,
    {
        *self.pure_write().await = T::DEFAULT;
    }

    /// Register an async callback which executes whenever the table has been updated.
    ///
    /// There is an internal 10 ms debounce to ensure waiters are not spammed for batched updates.
    pub async fn run_notifier<Fut: Future>(&self, func: impl Fn() -> Fut) -> ! {
        let mut generation = 0;
        loop {
            _ = self
                .waiters
                .wait_for(|| {
                    let table_generation = self.generation.load(Ordering::Acquire);
                    let updated = table_generation != generation;
                    generation = table_generation;
                    updated
                })
                .await;

            embassy_time::Timer::after_millis(10).await;

            // If the generation is stable, assume the parameter changes are finished.
            if self.generation.load(Ordering::Acquire) == generation {
                func().await;
            }
        }
    }

    /// Obtain a read-only lock on the table, loading persisted values first.
    ///
    /// This only needs to be called for the first reading of the table. Once it
    /// has loaded, subsequent reads return immediately from RAM.
    pub fn read(&'static self) -> impl Future<Output = TableReadGuard<'static, T>> {
        self.ensure_loaded().then(|_| self.pure_read())
    }
}

impl<T: ?Sized> ParamTable<T> {
    /// Get the name of this parameter table.
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Obtain a read-only lock on the table.
    pub fn pure_read(&self) -> impl Future<Output = TableReadGuard<'_, T>> {
        self.params.read().map(|guard| TableReadGuard { guard })
    }

    /// Obtain an exclusive write-capable lock on this table.
    pub fn pure_write(&self) -> impl Future<Output = TableWriteGuard<'_, T>> {
        self.params
            .write()
            .map(|guard| TableWriteGuard { guard, table: self })
    }

    /// Obtain a scoped writer lock on this table.
    pub fn scoped_write(&self, func: impl FnOnce(&mut T)) -> impl Future<Output = ()> {
        self.pure_write().map(|mut table| func(&mut *table))
    }

    /// Notify anyone waiting, that one or more values in this table has been updated.
    pub fn notify(&self) {
        self.generation.fetch_add(1, Ordering::Release);
        self.waiters.wake_all();
    }

    /// Ensure that persisted values for this table have been loaded into RAM.
    ///
    /// This is idempotent and safe to call concurrently: the first caller
    /// performs the load, while any others wait for it to complete.
    pub async fn ensure_loaded(&self) {
        match self.load_state.compare_exchange(
            UNLOADED,
            LOADING,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => {
                use super::task::{Request, request};
                let _ = request(Request::LoadTable(self.name)).await;
                self.load_state.store(LOADED, Ordering::Release);
                self.waiters.wake_all();
            }
            // Already loaded.
            Err(LOADED) => {}
            // Already _being_ loaded
            Err(_) => {
                _ = self
                    .waiters
                    .wait_for(|| self.load_state.load(Ordering::Acquire) == LOADED)
                    .await;
            }
        }
    }
}

impl ParamTable<dyn mav_param::Node> {
    /// Get the number of values currently present in this table.
    ///
    /// This does not reflect the maximum number of possible values, but only the current set.
    /// This should only be considered a hint, since the table may be modified after calling this function.
    pub async fn size_hint(&self) -> usize {
        let lock = self.pure_read().await;
        mav_param::param_iter(&*lock)
            .filter(|entry| entry.is_ok())
            .count()
    }
}
