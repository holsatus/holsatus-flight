use core::num::NonZeroU16;

use mavio::dialects::common::{
    enums::MavResult,
    messages::{CommandInt, CommandLong},
};

use super::super::{Generator, Request};

pub enum AnyCommand {
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

/// Make a request to the `Commander` task, converting its response into the equivalent `MavResult`
async fn mav_commander_request(command: crate::tasks::commander::message::Command) -> MavResult {
    use crate::tasks::commander::{
        message::{Origin, Request, Response},
        PROCEDURE,
    };

    let request = Request {
        origin: Origin::GroundControl,
        command,
    };

    let response = PROCEDURE.request(request).await;

    match response {
        Some(Response::Accepted) => MavResult::Accepted,
        Some(Response::Unchanged) => MavResult::Accepted,
        Some(Response::Unavailble) => MavResult::TemporarilyRejected,
        Some(Response::Rejected) => MavResult::TemporarilyRejected,
        Some(Response::Unsupported) => MavResult::Unsupported,
        Some(Response::Failed) => MavResult::Failed,
        None => MavResult::Cancelled,
    }
}

pub async fn set_message_interval(cmd: AnyCommand) -> MavResult {
    let message_id: u32 = cmd.param1() as u32;
    let period_us = cmd.param2() as u16;

    let Ok(generator) = Generator::try_from(message_id) else {
        return MavResult::Failed;
    };

    // Reject too fast intervals (1000 Hz is the fastest)
    if period_us < 1000 {
        return MavResult::Failed;
    }

    let request = match NonZeroU16::try_from(period_us) {
        Ok(period_ms) => Request::StartStream {
            generator,
            period_ms,
        },
        Err(_) => Request::StopStream { generator },
    };

    crate::mavlink::MAV_REQUEST.send(request).await;
    MavResult::Accepted
}

pub async fn do_set_actuator(cmd: AnyCommand) -> MavResult {
    use crate::tasks::commander::message::Command;

    let values = [cmd.param1(), cmd.param2(), cmd.param3(), cmd.param4()];
    let command = Command::SetActuators { group: 0, values };
    mav_commander_request(command).await
}

pub async fn component_arm_disarm(cmd: AnyCommand) -> MavResult {
    // Very strange to use floats at bools, but hey
    let arm = cmd.param1() as u8 == 1;
    let disarm = cmd.param1() as u8 == 0;
    let force = cmd.param2() as u16 == 21196;

    if !arm && !disarm {
        return MavResult::Failed;
    }

    use crate::tasks::commander::message::ArmDisarm;
    let command = ArmDisarm { arm, force };
    mav_commander_request(command.into()).await
}

pub async fn request_message(cmd: AnyCommand) -> MavResult {
    let message_id = cmd.param1() as u32;

    let Ok(generator) = message_id.try_into() else {
        return MavResult::Unsupported;
    };

    // TODO - Not done yet. In addition to the requested message, the
    // AUTOPILOT_VERSION should also be sent for some reason, see:
    // https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION

    crate::mavlink::MAV_REQUEST
        .send(Request::Single { generator })
        .await;
    crate::mavlink::MAV_REQUEST
        .send(Request::Single {
            generator: Generator::AutopilotVersion,
        })
        .await;

    MavResult::Accepted
}
