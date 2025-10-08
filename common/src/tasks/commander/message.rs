/// The command to be sent to the [`Commander`](crate::commander::Commander)
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    ArmDisarm(ArmDisarm),

    /// Start the calibration of the accelerometer(s)
    /// If `sensor_id` is `None`, all accelerometers will be calibrated
    /// otherwise only the accelerometer with the specified ID is calibrated
    ///
    /// Since this is a long-running operating handled by a separate task, the
    /// response will be `Response::Accepted` if the calibration was started
    /// successfully. The completion of the calibration will be indicated by
    /// a value produced by the calibrator.
    DoCalibration(DoCalibration),
    SetActuators {
        group: u8,
        values: [f32; 4],
    },
    SetActuatorOverride {
        active: bool,
    },
    SetControlMode {
        mode: crate::types::control::ControlMode
    },
    RunArmChecks,
}

macro_rules! impl_command_from {
    ($command:ident) => {
        impl From<$command> for super::Command {
            fn from(value: $command) -> Self {
                super::Command::$command(value)
            }
        }
    };
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ArmDisarm {
    pub arm: bool,
    pub force: bool,
}

impl_command_from!(ArmDisarm);

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DoCalibration {
    pub sensor_id: Option<u8>,
    pub sensor_type: SensorType,
}

impl_command_from!(DoCalibration);

/// A request to the [`Commander`](crate::commander::Commander)
pub struct Request {
    pub command: Command,
    pub origin: Origin,
}

impl <T: Into<Command>> From<(T, Origin)> for Request {
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
