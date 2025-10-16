use maitake_sync::{blocking::DefaultMutex, WaitQueue};
use mutex::{BlockingMutex, ConstInit, ScopedRawMutex};

pub struct Watch<T, M: ScopedRawMutex = DefaultMutex> {
    state: BlockingMutex<M, State<T>>,
    wait: WaitQueue<M>,
}

impl<T: Clone, M: ScopedRawMutex + ConstInit> Default for Watch<T, M> {
    fn default() -> Self {
        Self::new()
    }
}

struct State<T> {
    value: Option<T>,
    msg_id: usize,
}

impl<T: Clone, M: ScopedRawMutex> Watch<T, M> {
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            wait: WaitQueue::new_with_raw_mutex(M::INIT),
            state: BlockingMutex::new(State {
                value: None,
                msg_id: 0,
            }),
        }
    }

    pub const fn sender(&self) -> Sender<'_, T, M> {
        Sender { watch: self }
    }

    pub const fn receiver(&self) -> Receiver<'_, T, M> {
        Receiver {
            watch: self,
            msg_id: 0,
        }
    }

    pub fn try_get(&self) -> Option<T> {
        self.inner_getter(None, None).map(|(value, _)| value)
    }

    pub fn is(&self, other: &T) -> bool
    where
        T: PartialEq,
    {
        self.state
            .with_lock(|state| state.value.as_ref().is_some_and(|inner| inner == other))
    }

    pub fn try_get_and(&self, mut pred: impl FnMut(&T) -> bool) -> Option<T> {
        self.inner_getter(None, Some(&mut pred))
            .map(|(value, _)| value)
    }

    pub fn get_msg_id(&self) -> usize {
        self.state.with_lock(|state| state.msg_id)
    }

    pub fn send(&self, value: T) {
        self.state.with_lock(|state| {
            state.msg_id = state.msg_id.wrapping_add(1);
            state.value = Some(value);
        });
        self.wait.wake_all();
    }

    fn modify(&self, value: impl FnOnce(&mut T)) {
        self.state.with_lock(|state| {
            if let Some(inner) = &mut state.value {
                state.msg_id = state.msg_id.wrapping_add(1);
                value(inner);
            }
        });
        self.wait.wake_all();
    }

    fn inner_getter(
        &self,
        msg_id: Option<usize>,
        pred: Option<&mut dyn FnMut(&T) -> bool>,
    ) -> Option<(T, usize)> {
        self.state.with_lock(|state| {
            // Check if the Watch's msg_id is newer than our own.
            //
            // Use wrapping sub instead of b.msg_id > msg_id, to account for overflow, and to
            // ensure an overflowed subtraction will not panic. In theory, a receiver that is
            // _exactly_ 2^usize::BITS messages behind will be able to miss that particular
            // message. Though that seems extremely unlikely in any real application (right?)
            if msg_id.is_some_and(|msg_id| state.msg_id.wrapping_sub(msg_id) == 0) {
                return None;
            }

            // Extract the value if it exists, which it should if the check above succeeded.
            //
            // Though since the check above is more likely to fail in the average running program
            // we check that first, so we can return earlier.
            let Some(value) = &state.value else {
                return None;
            };

            // Finally test whether the predicate function (if any) is happy
            if pred.is_some_and(|pred| !pred(value)) {
                return None;
            }

            Some((value.clone(), state.msg_id))
        })
    }
}

pub struct Receiver<'a, T, M: ScopedRawMutex = DefaultMutex> {
    watch: &'a Watch<T, M>,
    msg_id: usize,
}

impl<T: Clone, M: ScopedRawMutex> Receiver<'_, T, M> {
    pub async fn changed(&mut self) -> T {
        loop {
            // The `Wait` is guaranteed to get woken by a `wake_all`
            // even before awaiting it, as per the docs. So we make
            // the `Wake` here, check the `Watch`, and if it is not
            // ready, only then await the future.
            let wait_future = self.watch.wait.wait();
            match self.try_changed() {
                Some(changed_value) => return changed_value,
                _ => {
                    // This future should never fail to yield Result::Ok
                    let result = wait_future.await;
                    debug_assert!(result.is_ok())
                }
            }
        }
    }

    pub fn try_changed(&mut self) -> Option<T> {
        self.watch
            .inner_getter(Some(self.msg_id), None)
            .map(|(value, msg_id)| {
                self.msg_id = msg_id;
                value
            })
    }

    pub async fn changed_and(&mut self, mut pred: impl FnMut(&T) -> bool) -> T {
        loop {
            let wait_future = self.watch.wait.wait();
            match self.try_changed_and(&mut pred) {
                Some(changed_value) => return changed_value,
                _ => {
                    let result = wait_future.await;
                    debug_assert!(result.is_ok())
                }
            }
        }
    }

    pub fn try_changed_and(&mut self, mut pred: impl FnMut(&T) -> bool) -> Option<T> {
        self.watch
            .inner_getter(Some(self.msg_id), Some(&mut pred))
            .map(|(value, msg_id)| {
                self.msg_id = msg_id;
                value
            })
    }

    pub async fn get(&mut self) -> T {
        loop {
            let wait_future = self.watch.wait.wait();
            match self.try_get() {
                Some(changed_value) => return changed_value,
                _ => {
                    let result = wait_future.await;
                    debug_assert!(result.is_ok())
                }
            }
        }
    }

    pub fn try_get(&mut self) -> Option<T> {
        self.watch.inner_getter(None, None).map(|(value, msg_id)| {
            self.msg_id = msg_id;
            value
        })
    }

    pub async fn get_and(&mut self, mut pred: impl FnMut(&T) -> bool) -> T {
        loop {
            let wait_future = self.watch.wait.wait();
            match self.try_get_and(&mut pred) {
                Some(changed_value) => return changed_value,
                _ => {
                    let result = wait_future.await;
                    debug_assert!(result.is_ok())
                }
            }
        }
    }

    pub fn try_get_and(&mut self, mut pred: impl FnMut(&T) -> bool) -> Option<T> {
        self.watch
            .inner_getter(None, Some(&mut pred))
            .map(|(value, msg_id)| {
                self.msg_id = msg_id;
                value
            })
    }
}

pub struct Sender<'a, T, M: ScopedRawMutex = DefaultMutex> {
    watch: &'a Watch<T, M>,
}

impl<T: Clone, M: ScopedRawMutex> Sender<'_, T, M> {
    pub fn send(&mut self, value: T) {
        self.watch.send(value);
    }

    pub fn modify(&mut self, value: impl FnOnce(&mut T)) {
        self.watch.modify(value);
    }
}

#[cfg(test)]
mod tests {

    use core::{
        future::Future,
        pin::Pin,
        task::{Context, Poll},
    };

    use futures::task::SpawnExt;
    use mutex::raw_impls::cs::CriticalSectionRawMutex;

    use super::*;

    static TEST_NUMBERS: &[i32] = &[1, 3, 3, 7];

    pub fn yield_now() -> impl Future<Output = ()> {
        YieldNowFuture { yielded: false }
    }

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    struct YieldNowFuture {
        yielded: bool,
    }

    impl Future for YieldNowFuture {
        type Output = ();
        fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            if self.yielded {
                Poll::Ready(())
            } else {
                self.yielded = true;
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }

    async fn multi_yield(num: usize) {
        for _ in 0..num {
            yield_now().await
        }
    }

    #[test]
    fn test_async_changed() {
        static WATCH: Watch<i32, CriticalSectionRawMutex> = Watch::new();
        let mut spawner = futures_executor::LocalPool::new();

        // Receiver task
        spawner
            .spawner()
            .spawn(async {
                let mut receiver = WATCH.receiver();

                for expected in TEST_NUMBERS {
                    assert_eq!(receiver.changed().await, *expected);
                    multi_yield(5).await;
                }
            })
            .unwrap();

        // Sender task
        spawner
            .spawner()
            .spawn(async {
                let mut sender = WATCH.sender();

                for number in TEST_NUMBERS {
                    sender.send(*number);
                    multi_yield(10).await;
                }
            })
            .unwrap();

        spawner.run();
    }

    #[test]
    fn test_async_multi_changed() {
        static WATCH: Watch<i32, CriticalSectionRawMutex> = Watch::new();
        let mut spawner = futures_executor::LocalPool::new();

        // Receiver _multiple_ tasks
        for _ in 0..10 {
            spawner
                .spawner()
                .spawn(async {
                    let mut receiver = WATCH.receiver();

                    for expected in TEST_NUMBERS {
                        assert_eq!(receiver.changed().await, *expected);
                        multi_yield(5).await;
                    }
                })
                .unwrap();
        }

        // Sender task
        spawner
            .spawner()
            .spawn(async {
                let mut sender = WATCH.sender();

                for number in TEST_NUMBERS {
                    sender.send(*number);
                    multi_yield(10).await;
                }
            })
            .unwrap();

        spawner.run();
    }

    #[test]
    fn test_async_get() {
        static WATCH: Watch<i32, CriticalSectionRawMutex> = Watch::new();
        let mut spawner = futures_executor::LocalPool::new();

        // Send value _once_
        WATCH.sender().send(10);

        // Receiver task
        spawner
            .spawner()
            .spawn(async {
                let mut receiver = WATCH.receiver();

                // Without yielding
                for _ in 0..5 {
                    assert_eq!(receiver.get().await, 10);
                }

                // With yielding
                for _ in 0..5 {
                    assert_eq!(receiver.get().await, 10);
                    multi_yield(10).await
                }
            })
            .unwrap();

        spawner.run();
    }

    #[test]
    fn test_various() {
        static WATCH: Watch<i32, CriticalSectionRawMutex> = Watch::new();

        let mut sender = WATCH.sender();
        let mut receiver = WATCH.receiver();

        assert_eq!(receiver.try_changed(), None);
        assert_eq!(receiver.try_get(), None);

        sender.send(10);

        assert_eq!(receiver.try_changed(), Some(10));
        assert_eq!(receiver.try_changed(), None);
        assert_eq!(receiver.try_get(), Some(10));

        sender.send(20);

        assert_eq!(receiver.try_get(), Some(20));
        assert_eq!(receiver.try_changed(), None);

        sender.send(30);

        assert_eq!(receiver.try_get_and(|v| v > &25), Some(30));
        assert_eq!(receiver.try_get_and(|v| v > &35), None);
        assert_eq!(receiver.try_get_and(|v| v > &25), Some(30));

        sender.send(40);

        assert_eq!(receiver.try_changed_and(|v| v > &25), Some(40));
        assert_eq!(receiver.try_changed_and(|v| v > &35), None);
        assert_eq!(receiver.try_changed_and(|v| v > &25), None);
    }

    #[test]
    fn test_overflowing() {
        static WATCH: Watch<i32, CriticalSectionRawMutex> = Watch::new();

        let mut sender = WATCH.sender();
        let mut receiver = WATCH.receiver();

        // Send an initial value
        sender.send(10);

        // Force the msg_id to almost overflow
        WATCH.state.with_lock(|inner| inner.msg_id = usize::MAX - 5);

        // Make the receiver update its internal msg_id
        assert_eq!(receiver.try_changed(), Some(10));

        for i in 0..10 {
            sender.send(i);
            assert_eq!(receiver.try_changed(), Some(i));
        }
    }
}
