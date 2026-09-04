pub trait VehicleDefinition: 'static {
    type FlightMode;
    fn set_flight_mode(mode: Self::FlightMode);
}

#[cfg(feature = "multicopter")]
pub use crate::multicopter::Multicopter as Vehicle;

pub type FlightMode = <Vehicle as VehicleDefinition>::FlightMode;
