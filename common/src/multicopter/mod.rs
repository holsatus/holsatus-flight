pub mod attitude_control;
pub mod flight_mode;

pub struct Multicopter;

impl crate::vehicle::SetFlightMode for Multicopter {
    type FlightModeKind = flight_mode::Kind;

    fn set_flight_mode(mode: Self::FlightModeKind) {
        flight_mode::REQUEST_MODE.send(mode);
    }
}
