use core::num::NonZeroU16;

use mavio::{default_dialect::{enums::{MavCmd, MavFrame}, messages::CommandAck}, dialects::common::{
    enums::MavResult,
    messages::{CommandInt, CommandLong},
}, prelude::MaybeVersioned};

use crate::mavlink2::{GeneratorPeriod, params::Identity};

use super::super::{messages::Generator, Message, CHANNEL};

/// A combined representation of [`CommandLong`] and [`CommandLong`].
#[allow(unused)]
pub struct CommandUnion {
    pub target_system: u8,
    pub target_component: u8,
    pub command: MavCmd,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub extra: Extra
}

/// The idea is to decide on a per-command basis whether it makes sense to
/// support one or the other or both. Most commands never even make use of them.
#[allow(unused)]
pub enum Extra {
    Int {
        frame: MavFrame,
        x: i32,
        y: i32,
        z: f32,
    },
    Long {
        param5: f32,
        param6: f32,
        param7: f32,
    },
}

impl From<CommandLong> for CommandUnion {
    fn from(value: CommandLong) -> Self {
        CommandUnion {
            target_system: value.target_system,
            target_component: value.target_component,
            command: value.command,
            param1: value.param1,
            param2: value.param2,
            param3: value.param3,
            param4: value.param4,
            extra: Extra::Long { 
                param5: value.param5,
                param6: value.param6,
                param7: value.param7,
            }
        }
    }
}

impl From<CommandInt> for CommandUnion {
    fn from(value: CommandInt) -> Self {
        CommandUnion {
            target_system: value.target_system,
            target_component: value.target_component,
            command: value.command,
            param1: value.param1,
            param2: value.param2,
            param3: value.param3,
            param4: value.param4,
            extra: Extra::Int { 
                frame: value.frame,
                x: value.x,
                y: value.y,
                z: value.z,
            }
        }
    }
}

impl <V: MaybeVersioned> super::Handler<V> for CommandLong {
    async fn handle_inner(
        self,
        server: &mut crate::mavlink2::MavlinkServer,
        frame: mavio::Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        let command = CommandUnion::from(self);
        command.handle_inner(server, frame).await
    }
}

impl <V: MaybeVersioned> super::Handler<V> for CommandInt {
    async fn handle_inner(
        self,
        server: &mut crate::mavlink2::MavlinkServer,
        frame: mavio::Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        let command = CommandUnion::from(self);
        command.handle_inner(server, frame).await
    }
}

impl <V: MaybeVersioned> super::Handler<V> for CommandUnion {
    async fn handle_inner(
        self,
        server: &mut crate::mavlink2::MavlinkServer,
        frame: mavio::Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {

        let target = Identity {
            sys: self.target_system,
            com: self.target_component,
        };

        if target != server.param.id {
            debug!("[mavlink] Received command for {:?}, ignoring", target);
            return Ok(())
        }

        let source = Identity {
            sys: frame.system_id(),
            com: frame.component_id(),
        };

        let response = match self.command {
            MavCmd::SetMessageInterval => set_message_interval(&self).await,
            MavCmd::DoSetActuator => do_set_actuator(&self).await,
            MavCmd::ComponentArmDisarm => component_arm_disarm(&self).await,
            MavCmd::RequestMessage => request_message(&self, source).await,
            // TODO: MavCmd::DoSetHome => (right click menu in QGroundControl)
            _ => {
                warn!("[mavlink] Unsupported command ID: {}", self.command as u32);
                MavResult::Unsupported
            }
        };

        let message = CommandAck {
            command: self.command,
            result: response,
            progress: 100, // Long-running commands?
            result_param2: 0,
            target_system: frame.system_id(),
            target_component: frame.component_id(),
        };

        server.send_mav_message(&message, source.into()).await?;

        Ok(())
    }
}

/// Make a request to the `Commander` task, converting its response into the equivalent `MavResult`
async fn mav_commander_request(command: crate::tasks::commander::message::Command) -> MavResult {
    use crate::tasks::commander::{
        message::{Origin, Request, Response},
        PROCEDURE,
    };

    // Always assume the request came from a GC
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

pub async fn set_message_interval(cmd: &CommandUnion) -> MavResult {
    let message_id = cmd.param1 as u32;
    let period_us = cmd.param2 as i32;

    let Some(generator) = Generator::from_id(message_id) else {
        warn!("[mavlink] Generator for message id {} not available", message_id);
        return MavResult::Unsupported;
    };

    let period_ms = if period_us < 0 {
        GeneratorPeriod::Disable
    } else {
        let period_ms = (period_us / 1000) as u16;
        match NonZeroU16::try_from(period_ms).ok() {
            Some(ms) => GeneratorPeriod::Millis(ms),
            None => GeneratorPeriod::Default,
        }
    };

    let request = Message::StreamGenerator { 
        generator, 
        period_ms,
    };

    crate::mavlink2::CHANNEL.send(request).await;
    MavResult::Accepted
}

pub async fn do_set_actuator(cmd: &CommandUnion) -> MavResult {
    use crate::tasks::commander::message::Command;

    let values = [cmd.param1, cmd.param2, cmd.param3, cmd.param4];
    let command = Command::SetActuators { group: 0, values };
    mav_commander_request(command).await
}

pub async fn component_arm_disarm(cmd: &CommandUnion) -> MavResult {
    // Very strange to use floats at bools, but hey
    let arm = cmd.param1 as u8 == 1;
    let disarm = cmd.param1 as u8 == 0;
    let force = cmd.param2 as u16 == 21196;

    if !arm && !disarm {
        return MavResult::Failed;
    }

    use crate::tasks::commander::message::ArmDisarm;
    let command = ArmDisarm { arm, force };
    mav_commander_request(command.into()).await
}

pub async fn request_message(cmd: &CommandUnion, source: Identity) -> MavResult {
    let message_id = cmd.param1 as u32;

    let Some(generator) = Generator::from_id(message_id) else {
        warn!("[mavlink] Generator for message id {} not available", message_id);
        return MavResult::Unsupported;
    };

    // TODO - Not done yet. In addition to the requested message, the
    // AUTOPILOT_VERSION should also be sent for some reason, see:
    // https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION

    CHANNEL
        .send(Message::SendGenerator { 
            generator, 
            target: source.into() })
        .await;
    CHANNEL
        .send(Message::SendGenerator {
            generator: Generator::AutopilotVersion,
            target: source.into(),
        })
        .await;

    MavResult::Accepted
}
