use linkme::distributed_slice;
use mav_param::{Ident, Value};

use super::{ParamError, table::ParamTable};

/// Every parameter table linked into the firmware.
///
/// Tables are added to this slice at link time by the [`crate::param_table!`]
/// macro, which means there is no runtime registration step, no fixed capacity,
/// and no need for a subsystem to be executed before its table is discoverable.
///
/// A table is only included if the crate defining it is linked into the final
/// binary, which is exactly the desired behaviour for feature-gated subsystems.
#[distributed_slice]
pub static PARAM_TABLES: [&'static ParamTable<dyn mav_param::Node>];

/// Global view over all linked parameter tables.
///
/// This is a zero-sized handle: the tables themselves live in [`PARAM_TABLES`].
pub struct ParamRegistry;

pub static PARAM_REGISTRY: ParamRegistry = ParamRegistry;

impl ParamRegistry {
    /// Iterate over every parameter table known to the firmware.
    pub fn iter(&self) -> impl Iterator<Item = &'static ParamTable<dyn mav_param::Node>> {
        PARAM_TABLES.iter().copied()
    }

    /// Look up a table by its identifier (the fragment before the `.`).
    pub fn get_table(&self, name: &str) -> Option<&'static ParamTable<dyn mav_param::Node>> {
        self.iter().find(|table| table.name() == name)
    }

    pub async fn get_param(&self, raw_ident: &[u8; 16]) -> Result<Value, ParamError> {
        // Ensure the provided raw_identifier can be converted into a valid identifier
        let ident = Ident::try_from(raw_ident).map_err(|_| ParamError::InvalidIdentifier)?;

        // Split the identifier at the first fragment 'table_ident.param_ident'
        let (table_ident, param_ident) = ident
            .as_str()
            .split_once('.')
            .ok_or(ParamError::NoTableFragment)?;

        // Find the table matching the 'table_ident' specifier
        let table = self.get_table(table_ident).ok_or(ParamError::NoMachingTable)?;

        // Find the parameter 'param_ident' in the table
        let reader = table.pure_read().await;
        let value =
            mav_param::get_value(&*reader, param_ident).ok_or(ParamError::NoMachingParam)?;

        Ok(value)
    }

    pub async fn set_param(&self, raw_ident: &[u8; 16], value: Value) -> Result<Ident, ParamError> {
        // Ensure the provided raw_identifier can be converted into a valid identifier
        let ident = Ident::try_from(raw_ident).map_err(|_| ParamError::InvalidIdentifier)?;

        // Split the identifier at the first fragment 'table_ident.param_ident'
        let (table_ident, param_ident) = ident
            .as_str()
            .split_once('.')
            .ok_or(ParamError::NoTableFragment)?;

        // Find the table matching the 'table_ident' specifier
        let table = self.get_table(table_ident).ok_or(ParamError::NoMachingTable)?;

        // Find the parameter 'param_ident' in the table and set its value
        let mut writer = table.pure_write().await;
        mav_param::set_value(&mut *writer, param_ident, value).ok_or(ParamError::NoMachingParam)?;

        table.notify();

        Ok(ident)
    }
}

#[cfg(test)]
mod test {
    use mav_param::{Ident, Value};

    use crate::params::ParamError;

    #[derive(mav_param::Tree)]
    struct Data1 {
        entry1: f32,
        entry2: i32,
    }

    #[derive(mav_param::Tree)]
    struct Data2 {
        entry1: f32,
        entry2: i32,
    }

    crate::const_default!(Data1 => {
        entry1: 3.14,
        entry2: 69,
    });

    crate::const_default!(Data2 => {
        entry1: 13.37,
        entry2: 420,
    });

    // Each test uses its own table to avoid cross-test interference, since the
    // distributed slice is shared by the whole test binary.
    crate::param_table!(static T1 = "t1" for Data1);
    crate::param_table!(static T2 = "t2" for Data2);
    crate::param_table!(static T3 = "t3" for Data1);

    #[test]
    fn test_expected_outputs() {
        futures_executor::block_on(async move {
            let get = async |name: &str| {
                super::PARAM_REGISTRY
                    .get_param(Ident::from_str_truncated(name).as_raw())
                    .await
            };

            assert_eq!(get("t1.entry1").await, Ok(Value::F32(3.14)));
            assert_eq!(get("t1.entry2").await, Ok(Value::I32(69)));

            assert_eq!(get("t2.entry1").await, Ok(Value::F32(13.37)));
            assert_eq!(get("t2.entry2").await, Ok(Value::I32(420)));

            assert_eq!(get("t2.invalid").await, Err(ParamError::NoMachingParam));
            assert_eq!(get("invalid.val").await, Err(ParamError::NoMachingTable));
            assert_eq!(get("illegal").await, Err(ParamError::NoTableFragment));
        });
    }

    #[test]
    fn test_set_then_get_param() {
        futures_executor::block_on(async move {
            let set = async |name: &str, value: Value| {
                super::PARAM_REGISTRY
                    .set_param(Ident::from_str_truncated(name).as_raw(), value)
                    .await
            };

            let get = async |name: &str| {
                super::PARAM_REGISTRY
                    .get_param(Ident::from_str_truncated(name).as_raw())
                    .await
            };

            // Initial values are good
            assert_eq!(get("t3.entry1").await, Ok(Value::F32(3.14)));
            assert_eq!(get("t3.entry2").await, Ok(Value::I32(69)));

            // We change some stuff
            assert!(set("t3.entry1", Value::F32(1.23)).await.is_ok());
            assert!(set("t3.entry2", Value::I32(9000)).await.is_ok());

            // The change is visible
            assert_eq!(get("t3.entry1").await, Ok(Value::F32(1.23)));
            assert_eq!(get("t3.entry2").await, Ok(Value::I32(9000)));
        });
    }

    #[test]
    fn test_no_duplicate_table_names() {
        let tables = super::PARAM_TABLES;
        for (index, table) in tables.iter().enumerate() {
            for other in tables.iter().skip(index + 1) {
                assert_ne!(
                    table.name(),
                    other.name(),
                    "duplicate parameter table name linked into the binary"
                );
            }
        }
    }
}
