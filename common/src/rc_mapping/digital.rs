use serde::{Deserialize, Serialize};

use crate::{tasks::commander::message::Command, types::control::ControlMode};

use super::NUM_DIGITAL_BINDS;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum RcEvent {
    // So we do not need Option<RcEvent>
    Unbound = 0,

    // Arming / disarming
    ArmMotors,
    DisarmMotors,

    // Switching control mode
    SetControlModeRate,
    SetControlModeAngle,
    SetControlModeVelocity,

    CalibrateAcc,
    CalibrateGyr,
    CalibrateMag,
}

impl TryFrom<RcEvent> for Command {
    type Error = ();
    
    fn try_from(value: RcEvent) -> Result<Self, Self::Error> {
        use crate::tasks::commander::message::*;
        let command = match value {
            RcEvent::Unbound => return Err(()),
            RcEvent::ArmMotors => ArmDisarm {
                arm: true,
                force: true,
            }.into(),
            RcEvent::DisarmMotors => ArmDisarm {
                arm: false,
                force: true,
            }.into(),

            RcEvent::SetControlModeAngle => Self::SetControlMode {
                mode: ControlMode::Angle,
            },
            RcEvent::SetControlModeRate => Self::SetControlMode {
                mode: ControlMode::Rate,
            },
            RcEvent::SetControlModeVelocity => Self::SetControlMode {
                mode: ControlMode::Velocity,
            },

            RcEvent::CalibrateAcc => DoCalibration {
                sensor_type: SensorType::Accelerometer,
                sensor_id: None,
            }.into(),
            RcEvent::CalibrateGyr => DoCalibration {
                sensor_type: SensorType::Gyroscope,
                sensor_id: None,
            }.into(),
            RcEvent::CalibrateMag => DoCalibration {
                sensor_type: SensorType::Magnetometer,
                sensor_id: None,
            }.into(),
        };

        Ok(command)
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DigitalBind(pub [Option<(u16, RcEvent)>; NUM_DIGITAL_BINDS]);

impl DigitalBind {
    pub const fn new(binds: &[(u16, RcEvent)]) -> Self {
        if binds.len() > NUM_DIGITAL_BINDS {
            core::panic!("Cannot assign more bindings than what is allocated for");
        }

        let mut bindings = [None; NUM_DIGITAL_BINDS];
        let mut index = 0;
        while index < binds.len() {
            bindings[index] = Some(binds[index]);
            index += 1
        }

        DigitalBind(bindings)
    }
}
