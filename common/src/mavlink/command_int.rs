use approx::abs_diff_eq;

use crate::{errors::MavlinkError, signals::COMMANDER_REQUEST, tasks::commander::CmdRequest};

use super::MavRequest;

pub(super) async fn set_message_interval(
    cmd: mavlink::holsatus::COMMAND_INT_DATA,
) -> Result<(), MavlinkError> {
    let index = cmd.param1 as u32;
    let freq_param = cmd.param2;

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
    cmd: mavlink::holsatus::COMMAND_INT_DATA,
) -> Result<(), MavlinkError> {
    let speeds = [
        cmd.param1,
        cmd.param2,
        cmd.param3,
        cmd.param4,
    ];
    
    COMMANDER_REQUEST.send(CmdRequest::SetMotorSpeeds(speeds)).await;

    Ok(())
}

pub(super) async fn component_arm_disarm(
    cmd: mavlink::holsatus::COMMAND_INT_DATA,
) -> Result<(), MavlinkError> {
    const ARM_ON: f32 = 1.0;
    const ARM_OFF: f32 = 0.0;
    const FORCE_ARM: f32 = 21196.0;

    let arm = abs_diff_eq!(cmd.param1, ARM_ON, epsilon = 1e-6);
    let disarm = abs_diff_eq!(cmd.param1, ARM_OFF, epsilon = 1e-6);
    let force = abs_diff_eq!(cmd.param2, FORCE_ARM, epsilon = 1e-1);

    if !arm && !disarm {
        return Err(MavlinkError::MalformedMessage);
    }

    COMMANDER_REQUEST.send(CmdRequest::ArmMotors { arm, force }).await;

    Ok(())
}