use std::time::{Duration, Instant};

pub struct Ticker {
    expires_at: Instant,
    duration: Duration,
}

#[allow(unused)]
impl Ticker {
    /// Creates a new ticker that ticks at the specified duration interval.
    pub fn every(duration: Duration) -> Self {
        let expires_at = Instant::now() + duration;
        Self {
            expires_at,
            duration,
        }
    }

    /// Resets the ticker back to its original state.
    /// This causes the ticker to go back to zero, even if the current tick isn't over yet.
    pub fn reset(&mut self) {
        self.expires_at = Instant::now() + self.duration;
    }

    /// Reset the ticker at the deadline.
    /// If the deadline is in the past, the ticker will fire instantly.
    pub fn reset_at(&mut self, deadline: Instant) {
        self.expires_at = deadline + self.duration;
    }

    /// Resets the ticker, after the specified duration has passed.
    /// If the specified duration is zero, the next tick will be after the duration of the ticker.
    pub fn reset_after(&mut self, after: Duration) {
        self.expires_at = Instant::now() + after + self.duration;
    }

    /// Waits for the next tick.
    pub fn next(&mut self) {
        let duration = self.expires_at.checked_duration_since(Instant::now());

        if let Some(duration) = duration {
            std::thread::sleep(duration);
        }

        self.expires_at += self.duration;
    }
}
