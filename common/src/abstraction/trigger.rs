use futures::FutureExt;

/// Some event trigger to read a sensor. Can be an interrupt, a timer, or an in-software signal.
pub trait Trigger {
    fn next_trigger(&mut self) -> impl Future<Output = ()>;
}

impl Trigger for embassy_time::Ticker {
    fn next_trigger(&mut self) -> impl Future<Output = ()> {
        self.next()
    }
}

/// Type implementing the [`Trigger`] trait on the rising edge of a digital signal
pub struct OnRising<W>(pub W);

/// Type implementing the [`Trigger`] trait on the falling edge of a digital signal
pub struct OnFalling<W>(pub W);

impl<W: embedded_hal_async::digital::Wait> Trigger for OnRising<W> {
    fn next_trigger(&mut self) -> impl Future<Output = ()> {
        self.0.wait_for_rising_edge().map(|_| ())
    }
}

impl<W: embedded_hal_async::digital::Wait> Trigger for OnFalling<W> {
    fn next_trigger(&mut self) -> impl Future<Output = ()> {
        self.0.wait_for_falling_edge().map(|_| ())
    }
}
