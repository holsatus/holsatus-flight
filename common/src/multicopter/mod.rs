pub mod attitude_control;
pub mod flight_mode;

pub struct Multicopter;

impl crate::vehicle::VehicleDefinition for Multicopter {
    type FlightMode = flight_mode::Kind;

    fn set_flight_mode(mode: Self::FlightMode) {
        flight_mode::REQUEST_MODE.send(mode);
    }
}
