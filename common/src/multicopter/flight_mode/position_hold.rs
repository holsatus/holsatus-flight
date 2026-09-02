use super::{Action, Controls, EnterError, FlightMode};

/// Placeholder for a position-hold mode.
pub struct PositionHold;

impl FlightMode for PositionHold {
    async fn enter(_controls: &Controls) -> Result<Self, EnterError> {
        debug_assert!(false, "Flight mode PositionHold is not implemented");
        Err(EnterError::Unimplemented)
    }

    async fn step(&mut self) -> Action {
        unreachable!("PositionHold is not implemented and cannot be entered")
    }
}
