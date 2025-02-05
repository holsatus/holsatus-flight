use mavio::dialects::common::{enums::MavResult, messages::{CommandInt, CommandLong}};

use crate::errors::MavlinkError;

use super::MavRequest;

pub(super) enum AnyCommand {
    Int(CommandInt),
    Long(CommandLong),
}

impl From<CommandInt> for AnyCommand {
    fn from(cmd: CommandInt) -> Self {
        AnyCommand::Int(cmd)
    }
}

impl From<CommandLong> for AnyCommand {
    fn from(cmd: CommandLong) -> Self {
        AnyCommand::Long(cmd)
    }
}

macro_rules! impl_any_command {
    ($($entry:ident => $type:ty)+) => {
        impl AnyCommand {
            $(
                pub fn $entry(&self) -> $type {
                    match self {
                        AnyCommand::Int(cmd) => cmd.$entry,
                        AnyCommand::Long(cmd) => cmd.$entry,
                    }
                }
            )+
        }
    };
}

impl_any_command!(
    command => mavio::dialects::common::enums::MavCmd
    param1 => f32
    param2 => f32
    param3 => f32
    param4 => f32
);

fn to_mav_result(res: Option<bool>) -> MavResult {
    match res {
        Some(true) => MavResult::Accepted,
        Some(false) => MavResult::Denied,
        None => MavResult::Failed,
    }
}

pub(super) async fn set_message_interval(
    cmd: AnyCommand,
) -> MavResult {

    let index: u32 = cmd.param1() as u32;
    let period_us = cmd.param2() as u32;

    if super::msg_generator::MavSendable::try_from(index).is_err() {
        return MavResult::Failed;
    }

    // Reject too fast intervals (1000 Hz is the fastest)
    // Though 0 is also a valid value, it means "stop sending"
    if period_us < 1000 || period_us != 0 {
        return MavResult::Failed;
    }

    let message = MavRequest::Stream { id: index, period_us };

    super::MAV_REQUEST.send(message).await;

    MavResult::Accepted
}

pub(super) async fn do_set_actuator(
    cmd: AnyCommand,
) -> MavResult {
    use crate::tasks::commander::{COMMANDER_PROC, SetActuators};

    let speeds = [
        cmd.param1(),
        cmd.param2(),
        cmd.param3(),
        cmd.param4(),
    ];

    let req = SetActuators { speeds };
    to_mav_result(COMMANDER_PROC.request(req).await.ok())
}

pub(super) async fn component_arm_disarm(
    cmd: AnyCommand,
) -> MavResult {
    use crate::tasks::commander::{COMMANDER_PROC, ArmMotors};

    // Very strange to use floats at bools here, but hey
    let arm = cmd.param1() as u8 == 1;
    let disarm = cmd.param1() as u8 == 0;
    let force = cmd.param2() as u16 == 21196;

    if !arm && !disarm {
        return MavResult::Failed;
    }

    let req = ArmMotors { arm, force };
    to_mav_result(COMMANDER_PROC.request(req).await.ok())
}

pub(super) async fn request_message(
    cmd: AnyCommand,
) -> MavResult {

    debug!("Received request for message: {:?}", cmd.param1());
    // TODO - Not done yet. In addition to the requested message, the
    // AUTOPILOT_VERSION should also be sent for some reason, see:
    // https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION

    use super::msg_generator::MavSendable;

    match MavSendable::try_from(cmd.param1() as u32) {
        Ok(sendable) => {
            super::MAV_REQUEST.send(MavRequest::Single { id: sendable as u32 }).await;
            super::MAV_REQUEST.send(MavRequest::Single { id: MavSendable::AutopilotVersion as u32 }).await;
            MavResult::Accepted
        },
        Err(_) => MavResult::Failed,
    }
}
