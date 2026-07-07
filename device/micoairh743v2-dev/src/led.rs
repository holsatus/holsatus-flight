//! Three-color status LED state machine (PE2 green / PE3 red / PE4 blue).
//!
//! One task owns the LED pins and renders a mode that the rest of the
//! firmware publishes via [`set`]. The color code is deliberately simple
//! enough to recall on the field without notes:
//!
//!   DARK          booting / calibrating / waiting for init. Nothing is
//!                 wrong; the drone is just not ready yet. (RC-link status
//!                 has its own LED on the receiver, so no color here.)
//!   BLUE solid    ready to arm: preflight passed, disarmed, waiting for
//!                 SA (Mid=Manual, High=Auto) and throttle.
//!   GREEN blink   arm sequence running (~2.25 s governor window). Do not
//!                 raise the throttle yet.
//!   GREEN solid   armed and ready to fly -- push the throttle up.
//!   RED solid     an operator-fixable condition is blocking arming:
//!                 preflight switch/stick positions, auto-arm gates
//!                 (throttle, level, lateral reference), the throttle-
//!                 high-on-arm guard, or the on-ground gate.
//!   RED strobe    error: SD missing, boot aborts, Fault state, or a
//!                 severe battery tier (the strobe overrides every other
//!                 mode while the tier is severe -- the in-flight
//!                 forced-descent alarm).
//!
//! Solid renderings drop out for one 50 ms tick every 2 s as an executor-
//! liveness flick: a frozen solid color means the thread executor died.
//!
//! Writers: the flight binary's main (SD abort) and mission FSM (all other
//! modes), once per tick with last-write-wins semantics. The task samples
//! at 20 Hz, so sub-tick double-writes are invisible.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

use crate::resources::LedResources;

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum LedMode {
    /// Booting / calibrating. All dark.
    Init = 0,
    /// Preflight passed, disarmed, ready to arm. Blue solid.
    Ready = 1,
    /// Arm command accepted, governor arm sequence running. Green blink.
    Arming = 2,
    /// Armed and ready to fly. Green solid.
    Armed = 3,
    /// Operator-fixable condition blocks arming. Red solid.
    Blocked = 4,
    /// Fatal error / Fault state. Red strobe.
    Error = 5,
}

static MODE: AtomicU8 = AtomicU8::new(LedMode::Init as u8);

/// Publish the LED mode. Callable from any task; the LED task picks the
/// latest value up within one 50 ms tick.
pub fn set(mode: LedMode) {
    MODE.store(mode as u8, Ordering::Relaxed);
}

fn get() -> LedMode {
    match MODE.load(Ordering::Relaxed) {
        1 => LedMode::Ready,
        2 => LedMode::Arming,
        3 => LedMode::Armed,
        4 => LedMode::Blocked,
        5 => LedMode::Error,
        _ => LedMode::Init,
    }
}

#[embassy_executor::task]
pub async fn led_task(r: LedResources) -> ! {
    let mut green = Output::new(r.green, Level::Low, Speed::Low);
    let mut blue = Output::new(r.blue, Level::Low, Speed::Low);
    let mut red = Output::new(r.red, Level::Low, Speed::Low);

    // Severe battery tiers override the published mode with the red
    // strobe. Kept inside the task so the alarm cannot be lost to a
    // stale FSM write while the pack is collapsing.
    let mut bat_rcv = crate::battery::BATTERY_TIER
        .receiver()
        .expect("BATTERY_TIER receiver slot (led)");

    let mut tick: u32 = 0;
    loop {
        Timer::after_millis(50).await;
        tick = tick.wrapping_add(1);

        let severe = bat_rcv.try_get().map(|t| t.is_severe()).unwrap_or(false);
        let mode = if severe { LedMode::Error } else { get() };

        // Liveness flick: solid colors go dark for one tick every 2 s.
        let flick = tick.is_multiple_of(40);
        let (g, b, r_on) = match mode {
            LedMode::Init => (false, false, false),
            LedMode::Ready => (false, !flick, false),
            // 2.5 Hz green blink: 200 ms on / 200 ms off.
            LedMode::Arming => (tick % 8 < 4, false, false),
            LedMode::Armed => (!flick, false, false),
            LedMode::Blocked => (false, false, !flick),
            // 5 Hz red strobe: 100 ms on / 100 ms off.
            LedMode::Error => (false, false, tick % 4 < 2),
        };
        green.set_level(if g { Level::High } else { Level::Low });
        blue.set_level(if b { Level::High } else { Level::Low });
        red.set_level(if r_on { Level::High } else { Level::Low });
    }
}
