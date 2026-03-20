/// The command to be sent to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    ArmDisarm{
        arm: bool,
        force: bool,
    },

    /// Start the calibration of the accelerometer(s)
    /// If `sensor_id` is `None`, all accelerometers will be calibrated
    /// otherwise only the accelerometer with the specified ID is calibrated
    ///
    /// Since this is a long-running operating handled by a separate task, the
    /// response will be `Response::Accepted` if the calibration was started
    /// successfully. The completion of the calibration will be indicated by
    /// a value produced by the calibrator.
    DoCalibration {
        sensor_id: Option<u8>,
        sensor_type: SensorType,
    },
    SetActuators {
        group: u8,
        values: [f32; 4],
    },
    SetActuatorOverride {
        active: bool,
    },
    SetControlMode(ControlMode),
    RunArmChecks,
    EskfResetOrigin,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlMode {
    Rate,
    Angle,
    Velocity,
    Autonomous,
}

/// A request to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request {
    pub command: Command,
    pub origin: Origin,
}

impl<T: Into<Command>> From<(T, Origin)> for Request {
    fn from((command, origin): (T, Origin)) -> Self {
        Request {
            command: command.into(),
            origin,
        }
    }
}

// TODO Move to somewhere else
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SensorType {
    Accelerometer,
    Gyroscope,
    Magnetometer,
    Barometer,
}

/// The response to a command to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    /// The command requested an unsupported operation
    Unsupported,

    /// Some necessary resources are not available
    Unavailble,

    /// The command would have no effect on the system
    Unchanged,

    /// The command was accepted and processed appropriately
    Accepted,

    /// The comnmand was rejected due to the current state of the system
    Rejected,

    /// The some error occured while processing the command
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Origin {
    /// The command was sent by the user via the RC
    RemoteControl,

    /// The command was sent via a telemetry link
    GroundControl,

    /// The command is of unspecified origin
    Unspecified,

    /// The command was the result of a failsafe event
    Failsafe,

    /// The command was the result of a kill switch event
    KillSwitch,

    /// The command was issued by an automated event
    Automatic,

    /// The command was issued via a command line
    CommandLine,
}
