use approx::abs_diff_eq;
use mavlink::holsatus::{MavCmd, COMMAND_INT_DATA, COMMAND_LONG_DATA};

use crate::{errors::MavlinkError, signals::COMMANDER_REQUEST, tasks::commander::CmdRequest};

use super::MavRequest;

pub(super) enum AnyCommand {
    Int(COMMAND_INT_DATA),
    Long(COMMAND_LONG_DATA),
}

impl From<COMMAND_INT_DATA> for AnyCommand {
    fn from(cmd: COMMAND_INT_DATA) -> Self {
        AnyCommand::Int(cmd)
    }
}

impl From<COMMAND_LONG_DATA> for AnyCommand {
    fn from(cmd: COMMAND_LONG_DATA) -> Self {
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
    command => MavCmd
    param1 => f32
    param2 => f32
    param3 => f32
    param4 => f32
);

pub(super) async fn set_message_interval(
    cmd: AnyCommand,
) -> Result<(), MavlinkError> {

    let index = cmd.param1() as u32;
    let freq_param = cmd.param2();

    let message = if freq_param > 1. {
        MavRequest::Stream { id: index, freq: (1e6/freq_param) as u16 }
    } else if freq_param > -0.5 {
        MavRequest::Stop { id: index }
    } else {
        // TODO: This (-1.) should be "default rate" for the message
        MavRequest::Stop { id: index }
    };
    
    super::MAV_REQUEST.send(message).await;

    Ok(())
}

pub(super) async fn do_set_actuator(
    cmd: AnyCommand,
) -> Result<(), MavlinkError> {

    let speeds = [
        cmd.param1(),
        cmd.param2(),
        cmd.param3(),
        cmd.param4(),
    ];
    
    COMMANDER_REQUEST.send(CmdRequest::SetMotorSpeeds(speeds)).await;

    Ok(())
}

pub(super) async fn component_arm_disarm(
    cmd: AnyCommand,
) -> Result<(), MavlinkError> {

    const ARM_ON: f32 = 1.0;
    const ARM_OFF: f32 = 0.0;
    const FORCE_ARM: f32 = 21196.0;

    let arm = abs_diff_eq!(cmd.param1(), ARM_ON, epsilon = 1e-6);
    let disarm = abs_diff_eq!(cmd.param1(), ARM_OFF, epsilon = 1e-6);
    let force = abs_diff_eq!(cmd.param2(), FORCE_ARM, epsilon = 1e-1);

    if !arm && !disarm {
        return Err(MavlinkError::MalformedMessage);
    }

    COMMANDER_REQUEST.send(CmdRequest::ArmMotors { arm, force }).await;

    Ok(())
}

pub(super) async fn request_message(
    cmd: AnyCommand,
) -> Result<(), MavlinkError> {
    
    debug!("Received request for message: {:?}", cmd.param1());
    // TODO - Not done yet. In addition to the requested message, the
    // AUTOPILOT_VERSION should also be sent for some reason, see:
    // https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
    
    Ok(())
}
