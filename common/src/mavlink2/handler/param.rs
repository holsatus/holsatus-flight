use mav_param::{Ident, Value};
use mavio::{
    default_dialect::{
        enums::MavParamType,
        messages::{ParamRequestList, ParamRequestRead, ParamSet, ParamValue},
    },
    prelude::MaybeVersioned,
    Frame,
};

use crate::mavlink2::{params::PeerId, MavlinkServer};

pub fn value_from_mav_bytewise(param_value: f32, param_type: MavParamType) -> Option<Value> {
    use mav_param::{value::from_bytewise, Value};
    let value = match param_type {
        MavParamType::Uint8 => Value::U8(from_bytewise(param_value)),
        MavParamType::Int8 => Value::I8(from_bytewise(param_value)),
        MavParamType::Uint16 => Value::U16(from_bytewise(param_value)),
        MavParamType::Int16 => Value::I16(from_bytewise(param_value)),
        MavParamType::Uint32 => Value::U32(from_bytewise(param_value)),
        MavParamType::Int32 => Value::I32(from_bytewise(param_value)),
        MavParamType::Real32 => Value::F32(from_bytewise(param_value)),
        _ => return None,
    };

    Some(value)
}

pub fn value_into_mav_bytewise(value: Value) -> (f32, MavParamType) {
    use mav_param::{value::into_bytewise, Value};
    match value {
        Value::U8(v) => (into_bytewise(v), MavParamType::Uint8),
        Value::I8(v) => (into_bytewise(v), MavParamType::Int8),
        Value::U16(v) => (into_bytewise(v), MavParamType::Uint16),
        Value::I16(v) => (into_bytewise(v), MavParamType::Int16),
        Value::U32(v) => (into_bytewise(v), MavParamType::Uint32),
        Value::I32(v) => (into_bytewise(v), MavParamType::Int32),
        Value::F32(v) => (into_bytewise(v), MavParamType::Real32),
    }
}

impl<V: MaybeVersioned> super::Handler<V> for ParamRequestRead {
    async fn handle_inner(
        server: &mut MavlinkServer,
        msg: Self,
        frame: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        // Message was not intended for us, ignore it // TODO routing?
        if msg.target_system != server.param.id.sys || msg.target_component != server.param.id.com {
            debug!("[mavlink] Received a ParamRequestList not intended for us");
            return Ok(());
        }

        let Ok(ident) = Ident::try_from(&msg.param_id) else {
            error!(
                "[mavlink] The requested parameter identifier is not valid utf8: {:?}",
                msg.param_id
            );
            return Ok(());
        };

        match crate::tasks::param_storage::TABLES
            .get_param(msg.param_id)
            .await
        {
            Some(value) => {
                debug!(
                    "[mavlink] Read out the parameter {} as {:?}",
                    ident.as_str(),
                    value
                );

                let param_id = ident.as_raw().clone();
                let (param_value, param_type) = value_into_mav_bytewise(value);

                let message = ParamValue {
                    param_id,
                    param_value,
                    param_type,
                    param_count: u16::MAX,
                    param_index: u16::MAX,
                };

                let target = PeerId {
                    sys: frame.header().system_id(),
                    com: frame.header().component_id(),
                };

                server.send_mav_message(&message, target).await?;
            }
            None => {
                error!(
                    "[mavlink] Could not read the parameter with id: {:?}",
                    msg.param_id
                )
            }
        }

        Ok(())
    }
}

impl<V: MaybeVersioned> super::Handler<V> for ParamRequestList {
    async fn handle_inner(
        server: &mut MavlinkServer,
        msg: Self,
        frame: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        // Message was not intended for us, ignore it // TODO routing?
        if msg.target_system != server.param.id.sys || msg.target_component != server.param.id.com {
            debug!("[mavlink] Received a ParamRequestList not intended for us");
            return Ok(());
        }

        // Get a copy of the parameter tables (references)
        let tables = crate::tasks::param_storage::TABLES
            .tables
            .with_lock(|t| t.clone());

        // Not super nice to iterate through the tables twice
        let mut param_count = 0;
        for table in tables.iter().cloned() {
            param_count += table.num_values().await as u16;
        }

        // Construct a message for each parameter
        let mut param_index = 0;
        for table in tables {
            let read = table.params.read().await;
            for maybe_param in mav_param::param_iter_named(&*read, table.name) {
                match maybe_param {
                    Ok(param) => {
                        let param_id = param.ident.as_raw().clone();
                        let (param_value, param_type) = value_into_mav_bytewise(param.value);

                        let message = ParamValue {
                            param_id,
                            param_value,
                            param_type,
                            param_count,
                            param_index,
                        };

                        param_index += 1;

                        let target = PeerId {
                            sys: frame.header().system_id(),
                            com: frame.header().component_id(),
                        };

                        server.send_mav_message(&message, target).await?;
                    }
                    Err(_) => error!("[mavlink] A parameter could not be fetched"),
                }
            }
        }

        Ok(())
    }
}

impl<V: MaybeVersioned> super::Handler<V> for ParamSet {
    async fn handle_inner(
        server: &mut MavlinkServer,
        msg: Self,
        _: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        // Message was not intended for us, ignore it // TODO routing?
        if msg.target_system != server.param.id.sys || msg.target_component != server.param.id.com {
            debug!("[mavlink] Received a ParamRequestList not intended for us");
            return Ok(());
        }

        let Some(value) = value_from_mav_bytewise(msg.param_value, msg.param_type) else {
            error!("[mavlink] An invalid parameter type was used in ParamSet command");
            return Ok(());
        };

        match crate::tasks::param_storage::TABLES
            .set_param(msg.param_id, value, true)
            .await
        {
            Some(ident) => {
                debug!(
                    "[mavlink] Set the parameter {:?} to {:?}",
                    ident.as_str(),
                    value
                )
            }
            None => {
                error!(
                    "[mavlink] Could not set the parameter with id: {:?}",
                    msg.param_id
                )
            }
        }

        Ok(())
    }
}
