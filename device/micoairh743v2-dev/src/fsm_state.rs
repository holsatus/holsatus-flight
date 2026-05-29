//! Shared mission-FSM state, published for telemetry.
//!
//! The mission FSM (`mission_fsm_task` in the flight/test binaries) owns the
//! `FsmState` value and writes it via [`publish`] once per tick. Read-only
//! consumers -- today just `phoenix_telem`, which mirrors it to the Pi as a
//! `NAMED_VALUE_INT name="FSM"` line and in `HEARTBEAT.custom_mode` -- read it
//! back via [`current_code`]. Keeping the enum here (rather than local to the
//! binary) means the wire encoding has exactly one definition.
//!
//! Wire encoding (the integer that appears on the web stream):
//!   0 = GroundIdle   1 = Manual   2 = AutoTakeoff
//!   3 = AutoHover    4 = AutoLand 5 = Fault

use core::sync::atomic::{AtomicU8, Ordering};

/// Mission-FSM states. `#[repr(u8)]` so `as u8` is the stable wire code above.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FsmState {
    GroundIdle = 0,
    Manual = 1,
    AutoTakeoff = 2,
    AutoHover = 3,
    AutoLand = 4,
    Fault = 5,
}

impl FsmState {
    pub fn label(self) -> &'static str {
        match self {
            FsmState::GroundIdle => "GroundIdle",
            FsmState::Manual => "Manual",
            FsmState::AutoTakeoff => "AutoTakeoff",
            FsmState::AutoHover => "AutoHover",
            FsmState::AutoLand => "AutoLand",
            FsmState::Fault => "Fault",
        }
    }
}

/// Latest published state code. Defaults to `GroundIdle` (0), which is also the
/// FSM's boot state, so the value is correct before the first `publish`.
static CURRENT: AtomicU8 = AtomicU8::new(FsmState::GroundIdle as u8);

/// Publish the current FSM state. Called by the FSM each tick.
pub fn publish(state: FsmState) {
    CURRENT.store(state as u8, Ordering::Relaxed);
}

/// Read the current state's wire code. Called by telemetry builders.
pub fn current_code() -> u8 {
    CURRENT.load(Ordering::Relaxed)
}
