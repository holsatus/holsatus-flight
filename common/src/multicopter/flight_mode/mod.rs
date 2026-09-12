//! Flight mode framework.
//!
//! This module manages the single active flight mode of the vehicle. A flight
//! mode produces setpoints, such as attitude and thrust commands through a small,
//! explicit set of [`Controls`]. The appropriate controllers consumes those setpoints
//! without having to know which mode produced them.

use core::future::Future;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, with_timeout};
use futures::TryFutureExt;

use crate::{
    multicopter::attitude_control::{ATTITUDE_COMMAND, AttitudeCommand},
    signals::{THROTTLE_COMMAND, ThrottleCommand},
    sync::{
        procedure::Procedure,
        watch::{Receiver, Sender, Watch},
    },
    tasks::rc_binder::params::digital::Event,
};

macro_rules! flight_modes {
    (
        $(
            $(#[$attr:meta])*
            $vehicle_mode:ident => $mode:ident($ty:ty) $(= $index:literal)?
        ),* $(,)?
    ) => {
        /// A flight mode that can be selected as the active mode.
        #[derive(Debug, Clone, Copy, PartialEq)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(u8)]
        pub enum Kind {
            None = 0,
            $(
                $(#[$attr])*
                $mode $( = $index)?,
            )*
        }

        /// The active flight mode and its state.
        #[repr(u8)]
        pub enum State {
            None,
            $(
                $(#[$attr])*
                $mode($ty) $( = $index)?,
            )*
        }

        impl State {
            pub const fn kind(&self) -> Kind {
                match self {
                    State::None => Kind::None,
                    $(
                        $(#[$attr])*
                        State::$mode(..) => Kind::$mode,
                    )*
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
                    $(
                        $(#[$attr])*
                            Kind::$mode => {
                            let mode = <$ty>::enter(&CONTROLS).map_err(map_err).await?;
                            State::$mode(mode)
                        }
                    )*
                };

                Ok(state)
            }

            /// Run one step of the active mode, bounded by [`STEP_TIMEOUT`].
            pub async fn step(&mut self) -> Action {
                match self {
                    State::None => core::future::pending().await,
                    $(
                        $(#[$attr])*
                        State::$mode(mode) => mode.step().await,
                    )*
                }
            }
        }

        impl TryFrom<Event> for Kind {
            type Error = ();

            fn try_from(value: Event) -> Result<Self, Self::Error> {
                let kind = match value {
                    $(
                        $(#[$attr])*
                        Event::$vehicle_mode => Kind::$mode,
                    )*
                    _ => return Err(()),
                };

                Ok(kind)
            }
        }

        impl From<Kind> for Event {
            fn from(value: Kind) -> Self {
                match value {
                    Kind::None => Event::None,
                    $(
                        $(#[$attr])*
                        Kind::$mode => Event::$vehicle_mode,
                    )*
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

pub use position_hold::POSITION_SP;

mod descend;
mod position_hold;
mod rc_acrobatic;
mod rc_stabilized;
mod stabilized;

#[cfg(feature = "mpc")]
pub mod mpc_autonomous;

flight_modes! {
    FlightMode0 => Descend(descend::Descend),
    FlightMode1 => PositionHold(position_hold::PositionHold),
    FlightMode2 => RcAcrobatic(rc_acrobatic::RcAcrobatic),
    FlightMode3 => RcStabilized(rc_stabilized::RcStabilized),
    FlightMode4 => Stabilized(stabilized::Stabilized),

    #[cfg(feature = "mpc")]
    FlightMode5 => MpcAutonomous(mpc_autonomous::MpcAutonomous)
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
pub static REQUEST_MODE: Watch<Kind> = Watch::new();

pub static REQUEST_MODE_PROC: Procedure<Kind, bool, 1> = Procedure::new();

pub mod params {
    use crate::params::ParamTable;

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {}

    crate::const_default!(
        Params => Params {}
    );

    pub static TABLE: ParamTable<Params> = ParamTable::default("flm");
}

/// The single flight mode manager task.
pub struct FlightModeRunner<'a> {
    current_mode: State,
    recv_request: Receiver<'a, Kind>,
    send_current: Sender<'a, Kind>,
}

#[embassy_executor::task]
pub async fn main() -> ! {
    FlightModeRunner::new().await.run().await
}

impl FlightModeRunner<'_> {
    pub async fn new() -> Self {
        let _ = params::TABLE.read().await;

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
                        error!("[mc/flight_mode] {:?}", error);
                    }
                }
                Either::Second(action) => {
                    if let Err(error) = self.handle_action(action).await {
                        error!("[mc/flight_mode] {:?}", error);
                    }
                }
            }

            // Publish if the flight mode changed
            let curr_kind = self.current_mode.kind();
            if prev_kind != curr_kind {
                info!("[mc/flight_mode] Entered flight mode {:?}", curr_kind);
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

    /// Transition to a new mode, specified by the `target` argument.
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
