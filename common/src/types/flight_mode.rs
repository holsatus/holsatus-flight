pub enum FlightMode {
    Autonomous(Autonomous),
    Assisted(Assisted),
    Manual(Manual),
}

pub enum Autonomous {
    Follow,
    Hold,
    Land,
    Mission,
    Orbit,
    Offboard,
    Return,
    Takeoff,
}

pub enum Assisted {
    /// The vehicle will hold its position in 3D space, and can be
    /// guided using control inputs.
    Position,

    /// The vehicle will try to hold a constant altitude using GNSS,
    /// barometers, lidar or sonar sensors. Control inputs can be used
    /// to raise or lower the vehicle, as well as manouver in the
    /// horizontal plane.
    Altitude,
}

pub enum Manual {
    /// Angle mode will self-level the vehicle according to the
    /// control setpoints. Let go of the sticks, and the vehicle
    /// will return to be level with the horizon.
    Angle,

    /// Rate control, or acro mode, uses rate setpoints to control
    /// the vehicle. Common for freestyle flying.
    Rate,

    /// Control inputs are sent directly (optionally mixed) to the
    /// actuators. No controllers are involved to assist in stabilizing.
    Raw,
}
