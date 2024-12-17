use serde::{Deserialize, Serialize};

use crate::{tasks::commander::CmdRequest, types::control::ControlMode};

use super::NUM_DIGITAL_BINDS;

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RcEvent {
    StartMotorTest,
    StopMotorTest,
    ArmMotors,
    DisarmMotors,
    AngleMode,
    RateMode,
    CalibrateAcc,
    CalibrateGyr,
    CalibrateMag,
}

impl From<RcEvent> for CmdRequest {
    fn from(value: RcEvent) -> Self {
        match value {
            RcEvent::StartMotorTest => Self::MotorTest(crate::tasks::motor_test::MotorTest::Roll {
                steady_speed: 0.20,
                millis_steady: 4000,
                delta_speed: 0.1,
                millis_delta: 2000,
            }),
            RcEvent::StopMotorTest => Self::MotorTest(crate::tasks::motor_test::MotorTest::Stop),
            RcEvent::ArmMotors => Self::ArmMotors{arm: true, force: true},
            RcEvent::DisarmMotors => Self::ArmMotors{arm: false, force: true},
            RcEvent::AngleMode => Self::SetMode(ControlMode::Angle),
            RcEvent::RateMode => Self::SetMode(ControlMode::Rate),
            RcEvent::CalibrateAcc => Self::CalibrateAcc((Default::default(), Some(0))),
            RcEvent::CalibrateGyr => Self::CalibrateGyr((Default::default(), Some(0))),
            RcEvent::CalibrateMag => Self::CalibrateMag((Default::default(), Some(0))),
        }
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DigitalCfg(pub [Option<(u16, RcEvent)>; NUM_DIGITAL_BINDS]);
