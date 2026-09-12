mod registry;
mod storage;
mod table;
mod task;

pub use registry::PARAM_REGISTRY;
pub use table::{ParamTable, TableReadGuard, TableWriteGuard};
pub use task::{Request, Response, entry, request};

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
