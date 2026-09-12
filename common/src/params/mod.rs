mod registry;
mod storage;
mod table;
mod task;

#[doc(hidden)]
pub use linkme;
pub use registry::{PARAM_REGISTRY, PARAM_TABLES};
pub use table::{ParamTable, TableReadGuard, TableWriteGuard};
pub use task::{Request, Response, entry, load_all, load_all_task, request};

/// Declare a globally discoverable parameter table.
///
/// This creates a [`ParamTable`] static and registers it with the global
/// [`PARAM_TABLES`] distributed slice at link time. Because registration happens
/// while linking, the table is discoverable (e.g. through `MAVLink` or the shell)
/// before any task that owns it has executed.
///
/// The optional attributes are applied to both the table and its registration,
/// which is useful for feature gating:
///
/// ```ignore
/// crate::param_table!(pub static TABLE = "eskf" for Parameters);
///
/// crate::param_table!(
///     #[cfg(feature = "imu_count_2")]
///     pub static IMU1 = "imu1" for Params
/// );
/// ```
#[macro_export]
macro_rules! param_table {
    ($(#[$attr:meta])* $vis:vis static $ident:ident = $name:literal for $ty:ty) => {
        $(#[$attr])*
        $vis static $ident: $crate::params::ParamTable<$ty> =
            $crate::params::ParamTable::default($name);

        // A `const _` block gives each registration its own scope, so the
        // fixed `REGISTRATION` name cannot collide across tables (unlike a
        // module-level `static _`, which is not a valid item name).
        $(#[$attr])*
        const _: () = {
            #[$crate::params::linkme::distributed_slice($crate::params::PARAM_TABLES)]
            static REGISTRATION: &'static $crate::params::ParamTable<dyn ::mav_param::Node> =
                &$ident;
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
