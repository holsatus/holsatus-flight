use core::{
    ops::{Deref, DerefMut},
    sync::atomic::Ordering,
};

use futures::FutureExt;
use maitake_sync::{RwLock, RwLockReadGuard, RwLockWriteGuard, WaitQueue};
use portable_atomic::AtomicU8;

use crate::params::registry::{PARAM_REGISTRY, Registration};

pub struct ParamTable<T: ?Sized> {
    name: &'static str,
    generation: AtomicU8,
    waiters: WaitQueue,
    params: RwLock<T>,
}

pub struct TableReadGuard<'a, T: ?Sized>(RwLockReadGuard<'a, T>);
pub struct TableWriteGuard<'a, T: ?Sized>(RwLockWriteGuard<'a, T>);

impl<T: ?Sized> Deref for TableReadGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: ?Sized> Deref for TableWriteGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: ?Sized> DerefMut for TableWriteGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T: mav_param::Node> ParamTable<T> {
    pub const fn new(name: &'static str, data: T) -> Self {
        ParamTable {
            name,
            generation: AtomicU8::new(0),
            waiters: WaitQueue::new(),
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
        *self.params.write().await = T::DEFAULT;
        self.notify();
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

    /// Register this table within the global static registry.
    ///
    /// This must be called in order to make the table globally available
    pub async fn ensure_registration(&'static self) {
        if PARAM_REGISTRY.register(self) == Registration::Inserted {
            use super::task::{Request, request};
            request(Request::LoadTable(self.name())).await;
        }
    }

    /// Register this table globally and get a lock on its loaded contents.
    ///
    /// You only need to call this for the first reading of the table.
    /// Once it is registered, the table is globally discoverable.
    pub fn read(&'static self) -> impl Future<Output = TableReadGuard<'static, T>> {
        ParamTable::ensure_registered(self).then(|_| self.pure_read())
    }
}

impl<T: ?Sized> ParamTable<T> {
    /// Get the name of this parameter table.
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Obtain a read-only lock on the table.
    pub fn pure_read(&self) -> impl Future<Output = TableReadGuard<'_, T>> {
        self.params.read().map(TableReadGuard)
    }

    /// Obtain an exclusive write-capable lock on this table.
    pub fn pure_write(&self) -> impl Future<Output = TableWriteGuard<'_, T>> {
        self.params.write().map(TableWriteGuard)
    }

    /// Notify anyone waiting, that one or more values in this table has been updated.
    pub fn notify(&self) {
        self.generation.fetch_add(1, Ordering::Release);
        self.waiters.wake_all();
    }
}

impl ParamTable<dyn mav_param::Node> {
    /// Register this table within the global static registry.
    ///
    /// This must be called in order to make the table globally available
    pub async fn ensure_registered(&'static self) {
        if PARAM_REGISTRY.register(self) == Registration::Inserted {
            use super::task::{Request, request};
            request(Request::LoadTable(self.name())).await;
        }
    }

    /// Get the number of values currently present in this table.
    ///
    /// This does not reflect the maximum number of possible values, but only the current set.
    /// This should only be considered a hint, since the table may be modified after calling this function.
    pub async fn size_hint(&self) -> usize {
        let lock = self.pure_read().await;
        mav_param::param_iter(&*lock).count()
    }
}
