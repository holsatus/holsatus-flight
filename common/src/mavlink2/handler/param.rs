use mav_param::Value;
use mavio::{
    default_dialect::{
        enums::MavParamType,
        messages::{ParamRequestList, ParamRequestRead, ParamSet, ParamValue},
    },
    prelude::MaybeVersioned,
    Frame,
};

use crate::mavlink2::{params::Identity, MavlinkServer};

pub fn value_from_mav_bytewise(param_value: f32, param_type: MavParamType) -> Option<Value> {
    use mav_param::value::from_bytewise;
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
    use mav_param::value::into_bytewise;
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

async fn send_parameter_value(server: &mut MavlinkServer, target: Identity, value: mav_param::Value, raw_ident: &[u8; 16]) -> Result<(), crate::mavlink2::Error> {
        let param_id = raw_ident.clone();
        let (param_value, param_type) = value_into_mav_bytewise(value);

        let message = ParamValue {
            param_id,
            param_value,
            param_type,
            param_count: u16::MAX,
            param_index: u16::MAX,
        };

        server.send_mav_message(&message, target.into()).await?;

        Ok(())
}

impl<V: MaybeVersioned> super::Handler<V> for ParamRequestRead {
    async fn handle_inner(
        self,
        server: &mut MavlinkServer,
        frame: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {

       let target = Identity {
            sys: self.target_system,
            com: self.target_component,
        };

        if !server.param.id.is_target_of(target) {
            return Ok(());
        }

        match crate::tasks::param_storage::TABLES
            .get_param(&self.param_id)
            .await
        {
            Ok(value) => {
                send_parameter_value(server, Identity::from(&frame), value, &self.param_id).await?;
            }
            Err(error) => {
                match core::str::from_utf8(&self.param_id) {
                    Ok(ident) => error!("[mavlink] Could not get the parameter: {:?} utf8({})", error, ident),
                    _ => error!("[mavlink] Could not get the parameter: {:?} raw({:?})", error, self.param_id),
                }
            }
        }

        Ok(())
    }
}

impl<V: MaybeVersioned> super::Handler<V> for ParamRequestList {
    async fn handle_inner(
        self,
        server: &mut MavlinkServer,
        frame: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {

        let target = Identity {
            sys: self.target_system,
            com: self.target_component,
        };

        if !server.param.id.is_target_of(target) {
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

                        let target = Identity {
                            sys: frame.system_id(),
                            com: frame.component_id(),
                        };

                        server.send_mav_message(&message, target.into()).await?;
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
        self,
        server: &mut MavlinkServer,
        frame: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {

        let target_id = Identity {
            sys: self.target_system,
            com: self.target_component,
        };

        // Require total match for setting parameter
        if target_id != server.param.id {
            return Ok(());
        }

        let Some(value) = value_from_mav_bytewise(self.param_value, self.param_type) else {
            error!("[mavlink] An invalid parameter type was used in ParamSet command");
            return Ok(());
        };

        match crate::tasks::param_storage::TABLES
            .set_param(&self.param_id, value)
            .await
        {
            Ok(ident) => {
                debug!(
                    "[mavlink] Set the parameter {:?} to {:?}",
                    ident.as_str(),
                    value
                );
            }
            Err(error) => {
                error!("[mavlink] Could not set the parameter: {:?}", error,);
                return Err(crate::mavlink2::Error::MaxNumberPorts) // TODO Correct error?
            }
        }

        match crate::tasks::param_storage::TABLES
            .get_param(&self.param_id)
            .await
        {
            Ok(value) => {
                send_parameter_value(server, Identity::from(&frame), value, &self.param_id).await?;
            }
            Err(error) => {
                error!("[mavlink] Could not get the parameter: {:?}", error,);
                return Err(crate::mavlink2::Error::MaxNumberPorts) // TODO Correct error?
            }
        }

        Ok(())
    }
}
