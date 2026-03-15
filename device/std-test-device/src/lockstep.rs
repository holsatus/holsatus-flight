use core::task::Waker;
use std::ptr::null_mut;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Mutex;

use embassy_executor::{raw, Spawner};
use embassy_time::Duration;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

/// Execute the task defined by `entry` in lockstep with the world model defined by the `world` closure.
///
/// # Panics
/// If called twice within the same process.
///
/// This function intended to be run in tests which reside in their own process.
/// The behavior of this function relies on global singletons being in a known initial state.
/// The function will panic if this is not the case.
pub fn lockstep_with(entry: impl FnOnce(Spawner), mut world: impl FnMut() -> Option<Duration>) {
    let executor: &'static raw::Executor = Box::leak(Box::new(raw::Executor::new(null_mut())));

    assert_eq!(
        DRIVER.ticks.swap(0, Ordering::Acquire),
        u64::MAX,
        "The time driver has been used twice in a single process"
    );

    entry(executor.spawner());

    while let Some(dt) = world() {
        unsafe { DRIVER.advance(dt.as_ticks(), executor) };
    }
}

static PENDING: AtomicBool = AtomicBool::new(true);

#[unsafe(no_mangle)]
fn __pender(_context: *mut ()) {
    PENDING.store(true, Ordering::Release);
}
pub struct LockstepDriver {
    ticks: AtomicU64,
    queue: Mutex<Queue>,
}

impl LockstepDriver {
    const fn new() -> Self {
        Self {
            // Sentinel value to indicate the driver has never been used
            ticks: AtomicU64::new(u64::MAX),
            queue: Mutex::new(Queue::new()),
        }
    }

    fn next_expiration(&self) -> u64 {
        self.queue.lock().unwrap().next_expiration(self.now())
    }

    /// Advances the simulation clock by a specific number of ticks,
    /// while completing all pending work within the executor.
    unsafe fn advance(&self, delta_ticks: u64, executor: &'static raw::Executor) {
        let target = self.now().saturating_add(delta_ticks.max(1));

        loop {
            // Deque expired timers and poll tasks to completion
            self.next_expiration();
            unsafe { executor.poll() };

            // Keep polling while work is being pended
            while PENDING.swap(false, Ordering::AcqRel) {
                unsafe { executor.poll() };
            }

            // Bump the tick count up to the next alarm (or target)
            let alarm = self.next_expiration();
            self.ticks.store(alarm.min(target), Ordering::Release);

            // We are done if the alarm exceeds target
            if alarm >= target {
                break;
            }
        }
    }
}

impl Driver for LockstepDriver {
    fn now(&self) -> u64 {
        self.ticks.load(Ordering::Acquire)
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let mut queue = self.queue.lock().unwrap();
        queue.schedule_wake(at, waker);
        queue.next_expiration(self.now());
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: LockstepDriver = LockstepDriver::new());
