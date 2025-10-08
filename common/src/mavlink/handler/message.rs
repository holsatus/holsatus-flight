use core::{future::Future, sync::atomic::Ordering};

use mavio::{
    dialects::common::{
        enums::{MavCmd, MavResult},
        messages,
    },
    prelude::V2,
    Frame,
};

use crate::{errors::MavlinkError, mavlink::handler::command};

use super::super::MavServer;

pub trait MessageHandler {
    fn handler(
        server: &MavServer,
        frame: Frame<V2>,
    ) -> impl Future<Output = Result<(), MavlinkError>>;
}

impl MessageHandler for messages::Heartbeat {
    async fn handler(_server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let _message = frame.decode_message::<Self>()?;
        let com_id = frame.header().component_id();
        let sys_id = frame.header().system_id();
        trace!("Received heartbeat from: {}, {}", sys_id, com_id);

        Ok(())
    }
}

impl MessageHandler for messages::Ping {
    async fn handler(server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let mut message = frame.decode_message::<Self>()?;
        debug!("Received ping request from system {}", frame.system_id());

        // Update the ping message to the senders ID
        message.target_component = frame.component_id();
        message.target_system = frame.system_id();
        server.write_message(&message).await?;

        Ok(())
    }
}

impl MessageHandler for messages::ParamSet {
    async fn handler(server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let message = frame.decode_message::<Self>()?;

        let bytes = f32::to_le_bytes(message.param_value);
        use mavio::dialects::common::enums::MavParamType;
        use mav_param::Value;

        let param = match message.param_type {
            MavParamType::Uint8 => Value::U8(u8::from_le_bytes([bytes[3]])),
            MavParamType::Int8 => Value::I8(i8::from_le_bytes([bytes[3]])),
            MavParamType::Uint16 => Value::U16(u16::from_le_bytes([bytes[2], bytes[3]])),
            MavParamType::Int16 => Value::I16(i16::from_le_bytes([bytes[2], bytes[3]])),
            MavParamType::Uint32 => Value::U32(u32::from_le_bytes(bytes)),
            MavParamType::Int32 => Value::I32(i32::from_le_bytes(bytes)),
            MavParamType::Real32 => Value::F32(message.param_value),
            _ => return Err(MavlinkError::Incompatible),
        };

        let vec = heapless::Vec::<_, 16>::from_slice(&message.param_id).expect("Vec is undersized");

        let string = heapless::String::from_utf8(vec).expect("Invalid utf8 param name");

        Ok(())
    }
}

impl MessageHandler for messages::RcChannelsOverride {
    async fn handler(server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let message = frame.decode_message::<Self>()?;

        // Ensure the message is for this system
        if message.target_system != server.inner.system_id.load(Ordering::Relaxed) {
            return Ok(());
        }

        let mut values = [990; 16];

        values[0] = message.chan1_raw;
        values[1] = message.chan2_raw;
        values[2] = message.chan3_raw;
        values[3] = message.chan4_raw;
        values[4] = message.chan5_raw;
        values[5] = message.chan6_raw;
        values[6] = message.chan7_raw;
        values[7] = message.chan8_raw;

        crate::signals::RC_CHANNELS_RAW.sender().send(Some(values));

        Ok(())
    }
}

impl MessageHandler for messages::CommandInt {
    async fn handler(server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let message = frame.decode_message::<Self>()?;

        // NOTE If any message can only be handled by one of the two
        // command message definitions, it should be handled here with
        // an early return. Otherwise, the message should be handled the
        // same for both COMMAND_INT and COMMAND_LONG.

        handle_any_command(server, &frame, message.into()).await?;

        Ok(())
    }
}

impl MessageHandler for messages::CommandLong {
    async fn handler(server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let message = frame.decode_message::<Self>()?;

        // NOTE If any message can only be handled by one of the two
        // command message definitions, it should be handled here with
        // an early return. Otherwise, the message should be handled the
        // same for both COMMAND_INT and COMMAND_LONG.

        handle_any_command(server, &frame, message.into()).await?;

        Ok(())
    }
}

async fn handle_any_command(
    server: &MavServer,
    frame: &Frame<V2>,
    message: command::AnyCommand,
) -> Result<(), MavlinkError> {
    let mav_cmd = message.command();
    debug!("Received command: {:?}", mav_cmd as u32);

    let result = match message.command() {
        MavCmd::SetMessageInterval => command::set_message_interval(message).await,

        MavCmd::DoSetActuator => command::do_set_actuator(message).await,

        MavCmd::ComponentArmDisarm => command::component_arm_disarm(message).await,

        MavCmd::RequestMessage => command::request_message(message).await,

        _ => MavResult::Unsupported,
    };

    let ack = messages::CommandAck {
        command: mav_cmd,
        result,
        progress: 100,
        result_param2: 0,
        target_system: frame.system_id(),
        target_component: frame.component_id(),
    };

    server.write_message(&ack).await?;

    Ok(())
}

impl MessageHandler for messages::Tunnel {
    async fn handler(_server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let message = frame.decode_message::<Self>()?;
        let slice = &message.payload[..message.payload_length as usize];
        debug!("Received tunneled data: {:?}", crate::logging::Bytes(slice));

        Ok(())
    }
}

impl MessageHandler for messages::MissionItemInt {
    async fn handler(_server: &MavServer, frame: Frame<V2>) -> Result<(), MavlinkError> {
        let _message = frame.decode_message::<Self>()?;
        debug!("Received MissingItemInt message");

        Ok(())
    }
}
