use crate::signals::ThrottleCommand;

pub mod attitude_control;
pub mod flight_mode;

pub struct Multicopter;

impl crate::vehicle::SetFlightMode for Multicopter {
    type FlightModeKind = flight_mode::Kind;

    fn set_flight_mode(mode: Self::FlightModeKind) {
        flight_mode::REQUEST_MODE.send(mode);
    }

    fn set_attitude_setpoint(att: nalgebra::UnitQuaternion<f32>) {
        flight_mode::stabilized::ATTITUDE_SETPOINT
            .send(attitude_control::AttitudeCommand::Angle(att));
    }

    fn set_thrust_setpoint(thrust: f32) {
        flight_mode::stabilized::THROTTLE_SETPOINT.send(ThrottleCommand(thrust));
    }

    fn set_angular_rate_setpoint(rate: [f32; 3]) {
        flight_mode::stabilized::ATTITUDE_SETPOINT
            .send(attitude_control::AttitudeCommand::Rate(rate.into()));
    }
}
