pub trait SetFlightMode {
    type FlightModeKind;
    fn set_flight_mode(mode: Self::FlightModeKind);
    fn set_attitude_setpoint(att: UnitQuaternion<f32>);
    fn set_angular_rate_setpoint(rate: [f32; 3]);
    fn set_thrust_setpoint(thrust: f32);
}

use nalgebra::UnitQuaternion;

pub type Vehicle = cfg_select! {
    feature = "multicopter" => crate::multicopter::Multicopter,
    feature = "fixedwing" => crate::fixedwing::Fixedwing, // TODO
    _ => compile_error!("A vehicle-type feature must be selected.")
};

pub type FlightModeKind = <Vehicle as SetFlightMode>::FlightModeKind;
