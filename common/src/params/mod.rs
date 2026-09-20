mod registry;
mod storage;
mod table;
mod task;

#[doc(hidden)]
pub use linkme;
pub use registry::{PARAM_REGISTRY, PARAM_TABLES};
pub use table::{ParamTable, TableReadGuard, TableWriteGuard};
pub use task::{Request, Response, entry, request};

/// Declare a globally discoverable parameter table.
///
/// This creates a [`ParamTable`] static and registers it with the global
/// [`PARAM_TABLES`] distributed slice at link time.
///
/// ```ignore
/// crate::param_table!(pub static TABLE: Params as "eskf");
///
/// crate::param_table!(
///     #[cfg(feature = "imu_count_2")]
///     pub static IMU1: Params as "imu1"
/// );
/// ```
#[macro_export]
macro_rules! param_table {
    (
        $(#[$attr:meta])*
        $vis:vis static $ident:ident: $ty:ty as $name:literal
    ) => {
        $(#[$attr])*
        $vis static $ident: $crate::params::ParamTable<$ty> =
            $crate::params::ParamTable::default($name);

        $(#[$attr])*
        const _: () = {
            // Link the table into the global slice of parameter tables
            #[$crate::params::linkme::distributed_slice($crate::params::PARAM_TABLES)]
            static REGISTRATION: &'static $crate::params::ParamTable<dyn ::mav_param::Node> =
                &$ident;

            // Use the linker to ensure parameter table names never clash.
            #[used]
            #[unsafe(export_name = ::core::concat!("__holsatus_param_table__", $name))]
            static PARAM_TABLE_NAME_GUARD: () = ();
        };
    };
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParamError {
    #[error("Invalid utf8 itentifier provided")]
    InvalidIdentifier,
    #[error("No table-level fragment specifier (.) found")]
    NoTableFragment,
    #[error("No table matched the given identifier")]
    NoMachingTable,
    #[error("No parameter matched the given identifier or type")]
    NoMachingParam,
}
