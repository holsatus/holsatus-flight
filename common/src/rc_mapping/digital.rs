use serde::{Deserialize, Serialize};

use crate::{tasks::commander::CmdRequest, types::control::ControlMode};

use super::NUM_DIGITAL_BINDS;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum RcEvent {

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

impl From<RcEvent> for CmdRequest {
    fn from(value: RcEvent) -> Self {
        match value {
            RcEvent::ArmMotors => Self::ArmMotors{arm: true, force: true},
            RcEvent::DisarmMotors => Self::ArmMotors{arm: false, force: true},

            RcEvent::SetControlModeAngle => Self::SetMode(ControlMode::Angle),
            RcEvent::SetControlModeRate => Self::SetMode(ControlMode::Rate),
            RcEvent::SetControlModeVelocity => Self::SetMode(ControlMode::Velocity),

            RcEvent::CalibrateAcc => Self::CalibrateAcc((Default::default(), Some(0))),
            RcEvent::CalibrateGyr => Self::CalibrateGyr((Default::default(), Some(0))),
            RcEvent::CalibrateMag => Self::CalibrateMag((Default::default(), Some(0))),
        }
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
