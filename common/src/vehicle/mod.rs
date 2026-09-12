
pub trait SetFlightMode {
    type FlightModeKind;
    fn set_flight_mode(mode: Self::FlightModeKind);
}

#[cfg(feature = "multicopter")]
pub use crate::multicopter::Multicopter as Vehicle;

pub type FlightModeKind = <Vehicle as SetFlightMode>::FlightModeKind;
