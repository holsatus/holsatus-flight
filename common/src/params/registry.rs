use heapless::Vec;
use maitake_sync::blocking::Mutex;
use mav_param::{Ident, Value};

use super::{ParamError, table::ParamTable};

pub struct ParamRegistry<'a, const N: usize> {
    pub tables: Mutex<Vec<&'a ParamTable<dyn mav_param::Node>, N>>,
}

// TODO: I would really like to avoid a fixed-sized registry.
// Some sort of linked-list with no removal should be doable and iterable

pub static PARAM_REGISTRY: ParamRegistry<15> = ParamRegistry::new();

#[derive(Debug, PartialEq)]
pub enum Registration {
    Full,
    Inserted,
    Duplicate,
}

impl<'a, const N: usize> ParamRegistry<'a, N> {
    pub const fn new() -> ParamRegistry<'a, N> {
        ParamRegistry {
            tables: Mutex::new(Vec::new()),
        }
    }

    pub fn get_table(&self, name: &str) -> Option<&'a ParamTable<dyn mav_param::Node>> {
        self.tables
            .with_lock(|tables| tables.iter().find(|table| table.name() == name).cloned())
    }

    /// Insert the given `Table` reference into the `TableSet`.
    ///
    /// Returns `true` if the table is inserted, and it is safe to wait for it to be read from storage.
    #[must_use]
    pub fn register(&self, table: &'a ParamTable<dyn mav_param::Node>) -> Registration {
        self.tables.with_lock(|tables| {
            if tables.iter().any(|t| core::ptr::addr_eq(*t, table)) {
                return Registration::Duplicate;
            }

            if tables.push(table).is_ok() {
                Registration::Inserted
            } else {
                error!(
                    "[param_registry] No more room to add table {}",
                    table.name()
                );
                Registration::Full
            }
        })
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
        let table = self
            .get_table(table_ident)
            .ok_or(ParamError::NoMachingTable)?;

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
        let table = self
            .get_table(table_ident)
            .ok_or(ParamError::NoMachingTable)?;

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

    use crate::params::{
        ParamError, ParamTable,
        registry::{ParamRegistry, Registration},
    };

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

    #[test]
    fn test_expected_outputs() {
        let table1 = ParamTable::<Data1>::default("t1");
        let table2 = ParamTable::<Data2>::default("t2");

        let registry = ParamRegistry::<10>::new();

        _ = registry.register(&table1);
        _ = registry.register(&table2);

        futures_executor::block_on(async move {
            let get = async |name: &str| {
                registry
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
        let table1 = ParamTable::<Data1>::default("t1");
        let registry = ParamRegistry::<10>::new();
        _ = registry.register(&table1);

        futures_executor::block_on(async move {
            let set = async |name: &str, value: Value| {
                registry
                    .set_param(Ident::from_str_truncated(name).as_raw(), value)
                    .await
            };

            let get = async |name: &str| {
                registry
                    .get_param(Ident::from_str_truncated(name).as_raw())
                    .await
            };

            // Initial values are good
            assert_eq!(get("t1.entry1").await, Ok(Value::F32(3.14)));
            assert_eq!(get("t1.entry2").await, Ok(Value::I32(69)));

            // We change some stuff
            assert!(set("t1.entry1", Value::F32(1.23)).await.is_ok());
            assert!(set("t1.entry2", Value::I32(9000)).await.is_ok());

            // The change is visible
            assert_eq!(get("t1.entry1").await, Ok(Value::F32(1.23)));
            assert_eq!(get("t1.entry2").await, Ok(Value::I32(9000)));
        });
    }

    #[test]
    fn test_registry_size_limit() {
        let table1 = ParamTable::<Data1>::default("t1");
        let table2 = ParamTable::<Data1>::default("t2");
        let table3 = ParamTable::<Data1>::default("t3");

        let registry = ParamRegistry::<2>::new();

        assert_eq!(registry.register(&table1), Registration::Inserted);
        assert_eq!(registry.register(&table1), Registration::Duplicate);
        assert_eq!(registry.register(&table2), Registration::Inserted);
        assert_eq!(registry.register(&table3), Registration::Full);
    }
}
