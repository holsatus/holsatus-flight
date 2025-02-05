use core::cell::RefCell;

use critical_section::Mutex;
use heapless::Deque;
use maitake_sync::WaitQueue;
use mutex::raw_impls::cs::CriticalSectionRawMutex;

pub struct MpscChannel<T, const N: usize> {
    state: Mutex<RefCell<Deque<T, N>>>,
    receive_queue: WaitQueue,
    send_queue: WaitQueue<CriticalSectionRawMutex>,
}

impl <T, const N: usize> MpscChannel<T, N> {

    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(Deque::new())),
            receive_queue: WaitQueue::new(),
            send_queue: WaitQueue::new_with_raw_mutex(CriticalSectionRawMutex::new()),
        }
    }

    pub async fn send_lazy(&self, mut func: impl FnOnce() -> T) {
        loop {
            let wait = self.send_queue.wait();
            match self.try_send_lazy(func) {
                Ok(()) => break,
                Err(retry) => {
                    let res = wait.await;
                    debug_assert!(res.is_ok());
                    func = retry;
                },
            }
        }
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

    pub fn try_send_lazy<F: FnOnce() -> T>(&self, value: F) -> Result<(), F> {
        let res = critical_section::with(|cs|{
            let mut deque = self.state.borrow_ref_mut(cs);
            if deque.is_full() {
                Err(value)
            } else {
                // SAFETY: We just checked that the deque is NOT full,
                // and we are inside a critical section, so this operation
                // is atomic. Also, pretty much identical to deque.push_back()
                unsafe { deque.push_back_unchecked(value()) }
                Ok(())
            }
        });

        if res.is_ok() {
            self.receive_queue.wake();
        }

        res
    }

    pub fn try_send(&self, value: T) -> Result<(), T> {
        let res = critical_section::with(|cs|{
            let mut deque = self.state.borrow_ref_mut(cs);
            deque.push_back(value)
        });

        if res.is_ok() {
            self.receive_queue.wake();
        }

        res
    }

    pub async fn receive(&self) -> T {
        loop {
            let wake = self.receive_queue.wait();
            match self.try_receive() {
                Some(value) => break value,
                None => {
                    let res = wake.await;
                    debug_assert!(res.is_ok())
                },
            }
        }
    }

    pub fn try_receive(&self) -> Option<T> {
        let res = critical_section::with(|cs|{
            self.state.borrow_ref_mut(cs).pop_front()
        });

        if res.is_some() {
            self.send_queue.wake()
        }

        res
    }
}
