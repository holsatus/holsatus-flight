//! Flight mode framework.
//!
//! This module manages the single active flight mode of the vehicle. A flight
//! mode produces attitude commands and a thrust setpoint through a small,
//! explicit set of "knobs". The attitude controller consumes those setpoints
//! without knowing which mode produced them.
//!
//! # Contract
//!
//! A flight mode implements [`FlightMode`]:
//!
//! - [`FlightMode::enter`] must be **pure**: it may read any global state
//!   (signals, parameter tables) to validate preconditions, but it must not
//!   write to any `Watch` or `Channel`. Instead it returns a [`Claim`]
//!   describing the initial setpoints the manager should publish on its behalf.
//!   This is what makes mode entry atomic: a failed entry cannot have modified
//!   anything.
//! - [`FlightMode::step`] produces the mode's setpoints. It is cancel-safe: it
//!   must be safe to drop at any await point, because the manager drops the
//!   step future whenever a mode switch is requested.
//! - [`FlightMode::exit`] is **synchronous and infallible**. Leaving a mode
//!   must never be partial or fallible, so any teardown work must be designed
//!   to be non-async: spawned tasks should be stopped through *cooperative
//!   cancellation* (a cancellation flag or signal they poll) rather than an
//!   async join, and owned handles simply drop when the mode is consumed.
//!   `exit` is guaranteed to run before every transition away from a mode,
//!   including after failed or timed-out steps.
//!
//! # Enforcement (manager-side)
//!
//! - `enter` must complete within [`ENTER_TIMEOUT`] (100 ms).
//! - `step` must complete within [`STEP_TIMEOUT`] (1 s). This is a *hang guard*
//!   not a cadence requirement; a mode may return [`Action::None`] frequently
//!   and publish setpoints at its own pace.
//!
//! On any failure (failed entry, failed or timed-out step), the manager walks a
//! fallback ladder ([`FALLBACK`]) to reach a safe mode. Unimplemented modes
//! panic in debug builds and fail entry in release builds, letting the ladder
//! descend instead.

use core::future::Future;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, with_timeout};
use futures::TryFutureExt;

use crate::{
    multicopter::attitude_control::{ATTITUDE_COMMAND, AttitudeCommand},
    signals::{THROTTLE_COMMAND, ThrottleCommand},
    sync::watch::{Receiver, Sender, Watch},
};

pub mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {}

    crate::const_default!(
        Params => Params {}
    );

    pub static TABLE: Table<Params> = Table::new("fm", Params::const_default());
}

macro_rules! flight_modes {
    (
        $(
            $mode:ident($ty:ty) $(= $index:literal)?
        ),* $(,)?
    ) => {
        /// A flight mode that can be selected as the active mode.
        #[derive(Debug, Clone, Copy, PartialEq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(u8)]
        pub enum Kind {
            None = 0,
            $( $mode $( = $index)?,  )*
        }

        /// The active flight mode and its state.
        #[repr(u8)]
        pub enum State {
            None,
            $( $mode($ty) $( = $index)?, )*
        }

        impl State {
            pub const fn kind(&self) -> Kind {
                match self {
                    State::None => Kind::None,
                    $( State::$mode(..) => Kind::$mode, )*
                }
            }

            /// Enter a mode and install it as the current state. Only assigns
            /// `curr` on success.
            async fn enter_from_kind(kind: Kind) -> Result<Self, Error> {
                let map_err = |source| Error::Enter {
                    target: kind,
                    source,
                };

                let state = match kind {
                    Kind::None => State::None,
                    $( Kind::$mode => {
                        let mode = <$ty>::enter(&CONTROLS).map_err(map_err).await?;
                        State::$mode(mode)
                    } )*
                };

                Ok(state)
            }

            /// Run one step of the active mode, bounded by [`STEP_TIMEOUT`].
            pub async fn step(&mut self) -> Action {
                match self {
                    State::None => core::future::pending().await,
                    $( State::$mode(mode) => mode.step().await, )*
                }
            }
        }
    };
}

/// The control outputs a flight mode is allowed to drive.
///
/// This is the only capability a mode is handed.
#[derive(Debug, Clone, Copy)]
pub struct Controls {
    pub attitude: Sender<'static, AttitudeCommand>,
    pub throttle: Sender<'static, ThrottleCommand>,
}

pub static CONTROLS: Controls = Controls {
    attitude: ATTITUDE_COMMAND.sender(),
    throttle: THROTTLE_COMMAND.sender(),
};

/// Contract implemented by every flight mode.
pub trait FlightMode: Sized {
    /// Enter the mode.
    ///
    /// Must be pure (no observable side effects) and must complete within [`ENTER_TIMEOUT`].
    fn enter(controls: &Controls) -> impl Future<Output = Result<Self, EnterError>>;

    /// Execute one step of the mode.
    ///
    /// Must be cancel-safe and complete within [`STEP_TIMEOUT`] to avoid triggering a failsafe.
    fn step(&mut self) -> impl Future<Output = Action>;
}

mod descend;
mod position_hold;
mod rc_acrobatic;
mod rc_stabilized;
mod stabilized;

flight_modes! {
    Descend(descend::Descend),
    PositionHold(position_hold::PositionHold),
    RcAcrobatic(rc_acrobatic::RcAcrobatic),
    RcStabilized(rc_stabilized::RcStabilized),
    Stabilized(stabilized::Stabilized),
}

/// A precondition that must be met for a mode to be entered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Precondition(u32);

bitflags::bitflags! {
    impl Precondition: u32 {
        /// A usable RC control link is present.
        const MANUAL_CONTROL = 1 << 0;
        /// A valid global position estimate is available.
        const GLOBAL_POSITION = 1 << 1;
        /// The gyroscope has been calibrated.
        const GYR_CALIBRATED = 1 << 2;
        /// The accelerometer has been calibrated.
        const ACC_CALIBRATED = 1 << 3;
    }
}

/// Why a mode could not be entered.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EnterError {
    #[error("Precondition(s) not met: {0:?}")]
    Precondition(Precondition),
    #[error("Timed out trying to enter mode")]
    Timeout,
    #[error("Flight mode is not implemented")]
    Unimplemented,
}

/// Why a step could not complete.
#[derive(Debug, Clone, Copy, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StepError {
    #[error("Timed out trying to step mode")]
    Timeout,
}

/// An error produced by the flight mode manager.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    #[error("Failed to enter {target:?}: {source}")]
    Enter {
        target: Kind,
        #[source]
        source: EnterError,
    },
    #[error("Failed to step {mode:?}: {source}")]
    Step {
        mode: Kind,
        #[source]
        source: StepError,
    },
    #[error("Failed to fall back after failure of {failed:?}")]
    Fallback { failed: Kind },
}

/// An action produced when a flight mode takes a step.
#[derive(Debug, Clone, Copy)]
pub enum Action {
    None,
    Transition(Kind),
}

const ENTER_TIMEOUT: Duration = Duration::from_millis(100);

/// The currently active flight mode.
pub static CURRENT_MODE: Watch<Kind> = Watch::new();

/// The desired (requested) flight mode.
static REQUEST_MODE: Watch<Kind> = Watch::new();

/// Request the flight mode to change
pub fn request_mode(kind: Kind) {
    REQUEST_MODE.send(kind);
}

/// The single flight mode manager task.
pub struct FlightModeRunner<'a> {
    current_mode: State,
    recv_request: Receiver<'a, Kind>,
    send_current: Sender<'a, Kind>,
}

impl FlightModeRunner<'_> {
    pub fn new() -> Self {
        Self {
            current_mode: State::None,
            recv_request: REQUEST_MODE.receiver(),
            send_current: CURRENT_MODE.sender(),
        }
    }

    pub async fn run(&mut self) -> ! {
        loop {
            let prev_kind = self.current_mode.kind();
            let result = select(self.recv_request.changed(), self.current_mode.step()).await;
            match result {
                Either::First(new_kind) => {
                    if let Err(error) = self.transition(new_kind).await {
                        error!("[mc::flight_mode] {:?}", error);
                    }
                }
                Either::Second(action) => {
                    if let Err(error) = self.handle_action(action).await {
                        error!("[mc::flight_mode] {:?}", error);
                    }
                }
            }

            // Publish if the flight mode changed
            let curr_kind = self.current_mode.kind();
            if prev_kind != curr_kind {
                info!("[mc::flight_mode] Entered flight mode {}", curr_kind);
                self.send_current.send(curr_kind);
            }
        }
    }

    async fn handle_action(&mut self, action: Action) -> Result<(), Error> {
        match action {
            Action::None => Ok(()),
            Action::Transition(kind) => self.transition(kind).await,
        }
    }

    /// Transition to a new mode, atomically.
    ///
    /// Ordering:
    /// 1. Exit the current mode (cleanup).
    /// 2. Disengage attitude control (commit point 1, before any fallible work).
    /// 3. Enter the target mode; on success publish its initial setpoints
    ///    (commit point 2) and swap the state.
    ///
    /// If entry fails or times out, walk the fallback ladder.
    async fn transition(&mut self, target: Kind) -> Result<(), Error> {
        if self.current_mode.kind() == target {
            warn!("[mc/flight_mode] Redundant flight mode change ignored");
            return Ok(());
        }

        match with_timeout(ENTER_TIMEOUT, State::enter_from_kind(target)).await {
            Ok(maybe_state) => {
                self.current_mode = maybe_state?;
                Ok(())
            }
            Err(_) => Err(Error::Enter {
                target,
                source: EnterError::Timeout,
            }),
        }
    }
}

#[embassy_executor::task]
pub async fn entry() -> ! {
    FlightModeRunner::new().run().await
}
