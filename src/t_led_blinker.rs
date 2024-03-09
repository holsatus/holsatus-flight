use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_sync::watch::DynReceiver;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn led_blinker(mut rcv_blinker_mode: DynReceiver<'static, BlinkerMode>, pin: AnyPin) {
    let mut blinker_mode = BlinkerMode::None;

    let mut led = Output::new(pin, Level::Low);
    'infinite: loop {
        match blinker_mode {
            BlinkerMode::None => {
                led.set_low();
                blinker_mode = rcv_blinker_mode.changed().await;
                continue 'infinite;
            }
            BlinkerMode::OneFast => one_fast(&mut led).await,
            BlinkerMode::TwoFast => two_fast(&mut led).await,
            BlinkerMode::ThreeFast => three_fast(&mut led).await,
            BlinkerMode::OnOffFast => on_off_fast(&mut led).await,
            BlinkerMode::OnOffSlow => on_off_slow(&mut led).await,
        };

        // Try to read new blinker mode
        if let Some(new_blinker_mode) = rcv_blinker_mode.try_changed() {
            blinker_mode = new_blinker_mode
        }
    }
}

async fn one_fast<'a>(led: &mut Output<'a>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(950)).await;
}

async fn two_fast<'a>(led: &mut Output<'a>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(800)).await;
}

async fn three_fast<'a>(led: &mut Output<'a>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;

    led.set_high(); // Short high
    Timer::after(Duration::from_millis(50)).await;

    led.set_low(); // Long low
    Timer::after(Duration::from_millis(650)).await;
}

async fn on_off_fast<'a>(led: &mut Output<'a>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(100)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(100)).await;
}

async fn on_off_slow<'a>(led: &mut Output<'a>) {
    led.set_high(); // Short high
    Timer::after(Duration::from_millis(500)).await;

    led.set_low(); // Medium low
    Timer::after(Duration::from_millis(500)).await;
}

#[derive(Clone, Copy)]
pub enum BlinkerMode {
    None,
    OneFast,
    TwoFast,
    ThreeFast,
    OnOffFast,
    OnOffSlow,
}
