use core::str::FromStr;

use crate::{
    errors::adapter::embedded_io::EmbeddedIoError,
    utils::u_types::{UBuffer, UFloat},
};
use embedded_cli::{
    arguments::{FromArgument, FromArgumentError},
    Command,
};
use embedded_io_async::{Read, Write};
use ufmt::{uWrite, uwrite};

pub struct ArgString<const N: usize> {
    pub string: heapless::String<N>,
}

impl<'a, const N: usize> FromArgument<'a> for ArgString<N> {
    fn from_arg(arg: &'a str) -> Result<Self, FromArgumentError<'a>>
    where
        Self: Sized,
    {
        let mut string = heapless::String::new();

        string.push_str(arg).map_err(|_| FromArgumentError {
            value: arg,
            expected: "Argument too long",
        })?;

        Ok(ArgString { string })
    }
}

struct UValue(mav_param::Value);

impl ufmt::uDisplay for UValue {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: uWrite + ?Sized,
    {
        match self.0 {
            mav_param::Value::U8(n) => uwrite!(f, "U8({})", n),
            mav_param::Value::I8(n) => uwrite!(f, "I8({})", n),
            mav_param::Value::U16(n) => uwrite!(f, "U16({})", n),
            mav_param::Value::I16(n) => uwrite!(f, "I16({})", n),
            mav_param::Value::U32(n) => uwrite!(f, "U32({})", n),
            mav_param::Value::I32(n) => uwrite!(f, "I32({})", n),
            mav_param::Value::F32(n) => uwrite!(f, "F32({})", UFloat(n, 4)),
        }
    }
}

#[derive(Clone, Copy)]
pub enum ArgValue {
    Float(f32),
    Int(i64),
}

impl<'a> FromArgument<'a> for ArgValue {
    fn from_arg(arg: &'a str) -> Result<Self, FromArgumentError<'a>>
    where
        Self: Sized,
    {
        match arg {
            "true" | "True" | "TRUE" => Ok(ArgValue::Int(1)),
            "false" | "False" | "FALSE" => Ok(ArgValue::Int(0)),
            _ if arg.contains('.') => {
                let number = FromStr::from_str(arg).map_err(|_| FromArgumentError {
                    value: arg,
                    expected: "Could not be parsed as float",
                })?;

                Ok(ArgValue::Float(number))
            }
            _ => {
                let number = FromStr::from_str(arg).map_err(|_| FromArgumentError {
                    value: arg,
                    expected: "Could not be parsed as integer",
                })?;

                Ok(ArgValue::Int(number))
            }
        }
    }
}

#[derive(Command)]
#[command(help_title = "Parameter commands")]
pub enum ParamCommand {
    /// List all active parameters in the system
    List,

    /// Get the value of a single parameter
    Get {
        /// The identifier/name of the parameter
        ident: ArgString<32>,
    },

    /// Set the value of a single parameter
    Set {
        /// The identifier/name of the parameter
        ident: ArgString<32>,

        /// The value of the parameter
        value: ArgValue,
    },

    /// Save the parameter to persistent storage
    Save {
        /// The identifier/name of the parameter
        ident: ArgString<32>,
    },

    /// Save all parameters to global storage
    SaveAll,
}

impl super::CommandHandler for ParamCommand {
    async fn handler(
        &self,
        mut serial: impl Read<Error = EmbeddedIoError> + Write<Error = EmbeddedIoError>,
    ) -> Result<(), EmbeddedIoError> {
        use crate::tasks::param_storage::TABLES;
        match self {
            ParamCommand::List => {
                serial
                    .write_all(b"Enumerating all system parameters..\n\r")
                    .await?;

                let tables = TABLES.tables.with_lock(|t| t.clone());

                for table in tables {
                    let read = table.params.read().await;
                    for maybe_param in mav_param::param_iter_named(&*read, table.name) {
                        match maybe_param {
                            Ok(param) => {
                                let mut buffer = UBuffer::<48>::new();
                                uwrite!(buffer, "{}", param.ident.as_str())?;
                                for _ in 0..17usize.saturating_sub(buffer.len()) {
                                    _ = buffer.push_byte(b' ');
                                }
                                let value = UValue(param.value);
                                uwrite!(buffer, ": {} \n\r", value)?;
                                serial.write_all(buffer.bytes()).await?;
                            }
                            Err(_) => {
                                serial
                                    .write_all(b"A parameter could not be fetched..\n\r")
                                    .await?
                            }
                        }
                    }
                }
            }
            ParamCommand::Get { ident } => {
                let Some((module, param)) = ident.string.split_once('.') else {
                    serial.write_all(b"[warn] not yet implemented\n\r").await?;
                    return Ok(());
                };

                let Some(table) = TABLES.get_table(module) else {
                    serial.write_all(b"[warn] No such table exists\n\r").await?;
                    return Ok(());
                };

                let table = table.params.read().await;

                let Some(table_value) = mav_param::get_value(&*table, param) else {
                    serial
                        .write_all(b"[warn] No such parameter exists\n\r")
                        .await?;
                    return Ok(());
                };

                let mut buffer = UBuffer::<32>::new();
                uwrite!(buffer, "{}", ident.string.as_str())?;
                for _ in 0..17usize.saturating_sub(buffer.len()) {
                    _ = buffer.push_byte(b' ');
                }
                let value = UValue(table_value);
                uwrite!(buffer, ": {} \n\r", value)?;
                serial.write_all(buffer.bytes()).await?;
            }
            ParamCommand::Set { ident, value } => {
                let Some((module, param)) = ident.string.split_once('.') else {
                    serial.write_all(b"[warn] not yet implemented\n\r").await?;
                    return Ok(());
                };

                let Some(table) = TABLES.get_table(module) else {
                    serial.write_all(b"[warn] No such table exists\n\r").await?;
                    return Ok(());
                };

                let mut table = table.params.write().await;

                let Some(table_value) = mav_param::get_value(&*table, param) else {
                    serial
                        .write_all(b"[warn] No such parameter exists\n\r")
                        .await?;
                    return Ok(());
                };

                macro_rules! match_values {
                    (
                        $(
                            $mav_var:ident, $arg_var:ident $(, $type:ident)?
                        );+ $(;)?
                    ) => {
                            match (table_value, *value) {
                            $(
                                (::mav_param::Value::$mav_var(_) , ArgValue::$arg_var(num)) $(
                                    if num > $type::MIN as i64 && num < $type::MAX as i64
                                )? => ::mav_param::Value::$mav_var(num $(as $type)?)
                            ),+ ,
                            _ => {
                                serial.write_all(b"[warn] Incompatible numeric type or value\n\r").await?;
                                return Ok(());
                            }
                        }
                    };
                }

                let new_value = match_values!(
                    U8, Int, u8;
                    I8, Int, i8;
                    U16, Int, u16;
                    I16, Int, i16;
                    U32, Int, u32;
                    I32, Int, i32;
                    F32, Float;
                );

                let written = mav_param::set_value(&mut *table, param, new_value).is_some();

                drop(table);

                if written {
                    serial
                        .write_all(b"[info] Parameter modified successfully\n\r")
                        .await?
                } else {
                    serial
                        .write_all(b"[error] Parameter could not be modified\n\r")
                        .await?
                }
            }
            ParamCommand::Save { ident } => {
                use crate::tasks::param_storage::{request, Request, Response};
                let ident = mav_param::Ident::from_str_truncated(&ident.string);
                match request(Request::SaveParam(ident)).await {
                    Some(Response::Success) => {
                        serial
                            .write_all(b"[info] Parameter succesfully saved\n\r")
                            .await?
                    }
                    Some(Response::Failure) => {
                        serial
                            .write_all(b"[error] Parameter could not be saved\n\r")
                            .await?
                    }
                    None => {
                        serial
                            .write_all(b"[error] Parameter storage module failed to respond\n\r")
                            .await?
                    }
                }
            }
            ParamCommand::SaveAll => {
                use crate::tasks::param_storage::{request, Request, Response};
                match request(Request::SaveAllTables).await {
                    Some(Response::Success) => {
                        serial
                            .write_all(b"[info] All parameters succesfully saved\n\r")
                            .await?
                    }
                    Some(Response::Failure) => {
                        serial
                            .write_all(b"[error] All parameters could not be saved\n\r")
                            .await?
                    }
                    None => {
                        serial
                            .write_all(b"[error] Parameter storage module failed to respond\n\r")
                            .await?
                    }
                }
            }
        }
        Ok(())
    }
}
