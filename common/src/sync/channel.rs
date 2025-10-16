use heapless::Deque;
use maitake_sync::{blocking::DefaultMutex, WaitQueue};
use mutex::{BlockingMutex, ConstInit, ScopedRawMutex};

pub struct Channel<T, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    state: BlockingMutex<M, Deque<T, N>>,
    recv_queue: WaitQueue<M>,
    send_queue: WaitQueue<M>,
}

impl<T, M: ScopedRawMutex + ConstInit, const N: usize> Default for Channel<T, N, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, M: ScopedRawMutex, const N: usize> Channel<T, N, M> {
    pub const fn new() -> Self
    where
        M: ConstInit,
    {
        Self {
            state: BlockingMutex::new(Deque::new()),
            recv_queue: WaitQueue::new_with_raw_mutex(M::INIT),
            send_queue: WaitQueue::new_with_raw_mutex(M::INIT),
        }
    }

    pub const fn receiver(&self) -> Receiver<'_, T, N, M> {
        Receiver { inner: self }
    }

    pub const fn sender(&self) -> Sender<'_, T, N, M> {
        Sender { inner: self }
    }

    pub fn try_send(&self, value: T) -> Result<(), T> {
        let res = self.state.with_lock(|deque| deque.push_back(value));

        if res.is_ok() {
            self.recv_queue.wake();
        }

        res
    }

    pub async fn send(&self, mut value: T) {
        loop {
            let wait = self.send_queue.wait();
            match self.try_send(value) {
                Ok(()) => break,
                Err(retry) => {
                    let res = wait.await;
                    debug_assert!(res.is_ok());
                    value = retry;
                }
            }
        }
    }

    pub fn try_receive(&self) -> Option<T> {
        let res = self.state.with_lock(|deque| deque.pop_front());

        if res.is_some() {
            self.send_queue.wake()
        }

        res
    }

    pub async fn receive(&self) -> T {
        loop {
            let wake = self.recv_queue.wait();
            match self.try_receive() {
                Some(value) => break value,
                None => {
                    let res = wake.await;
                    debug_assert!(res.is_ok())
                }
            }
        }
    }
}

pub struct Sender<'a, T, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    inner: &'a Channel<T, N, M>,
}

impl<T, M: ScopedRawMutex, const N: usize> Sender<'_, T, N, M> {
    pub async fn send(&self, value: T) {
        self.inner.send(value).await
    }

    pub fn try_send(&self, value: T) -> Result<(), T> {
        self.inner.try_send(value)
    }
}

pub struct Receiver<'a, T, const N: usize, M: ScopedRawMutex = DefaultMutex> {
    inner: &'a Channel<T, N, M>,
}

impl<T, M: ScopedRawMutex, const N: usize> Receiver<'_, T, N, M> {
    pub async fn receive(&self) -> T {
        self.inner.receive().await
    }

    pub fn try_receive(&self) -> Option<T> {
        self.inner.try_receive()
    }
}
