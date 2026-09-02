use super::{Action, Controls, EnterError, FlightMode};

/// Placeholder for a position-hold mode.
pub struct Descend;

impl FlightMode for Descend {
    async fn enter(_controls: &Controls) -> Result<Self, EnterError> {
        debug_assert!(false, "Flight mode Descend is not implemented");
        Err(EnterError::Unimplemented)
    }

    async fn step(&mut self) -> Action {
        unreachable!("Descend is not implemented and cannot be entered")
    }
}
