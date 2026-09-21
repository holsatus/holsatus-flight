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
/// ```
/// /// Define the struct of parameters using `mav_param`
/// #[derive(mav_param::Tree)]
/// struct Params {
///     my_param: u32
/// }
///
/// /// Ensure we can const-default initialize the struct
/// common::const_default! {
///     Params => { my_param: 69 }
/// };
///
/// /// Use this macro to define the table and its namespace
/// common::param_table!(pub static TABLE: Params as "name");
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
    #[error("Invalid UTF-8 identifier provided")]
    InvalidIdentifier,
    #[error("No table-level fragment specifier (.) found")]
    NoTableFragment,
    #[error("No table matched the given identifier")]
    NoMachingTable,
    #[error("No parameter matched the given identifier or type")]
    NoMachingParam,
}
