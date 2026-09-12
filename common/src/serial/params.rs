use crate::{params::ParamTable, serial::StreamId};

#[derive(mav_param::Tree, Clone)]
pub struct Parameters {
    pub assign: [Option<Assigned>; super::MAX_IO_STREAMS],
}

#[derive(mav_param::Tree, Default, Clone)]
pub struct Assigned {
    pub stream: StreamId,
    pub task: TaskId,
}

// io.assign.0.task = 2;
// io_port.0.task = 2;

/// The identifier for a task.
///
/// Note: 0 is a "special value" that represents no task.
#[derive(mav_param::Node, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TaskId(u32);

#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TaskName(pub &'static str);

impl TaskId {
    pub const fn new(name: &str) -> TaskId {
        TaskId(fnv1a_hash_u32(name))
    }
}

crate::const_default!(Parameters => {
    assign: [const { None }; super::MAX_IO_STREAMS],
});

pub static TABLE: ParamTable<Parameters> = ParamTable::default("io");

pub const fn fnv1a_hash_u32(s: &str) -> u32 {
    let hash = fnv1a_hash(s);
    ((hash >> 32) ^ hash & 0xFFFFFFFF) as u32
}

pub const fn fnv1a_hash(s: &str) -> u64 {
    const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x100000001b3;
    let bytes = s.as_bytes();
    let mut hash = FNV_OFFSET_BASIS;
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
        i += 1;
    }
    hash
}
