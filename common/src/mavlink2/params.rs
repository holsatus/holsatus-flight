use embassy_time::Duration;
use mavio::{prelude::MaybeVersioned, protocol::Frame};

use crate::{mavlink2::messages::Generator, tasks::param_storage::Table};

#[derive(mav_param::Tree, Clone)]
pub struct Parameters {
    /// The MAVLink identity of this system + component
    pub id: Identity,
    /// Duration of time with no messages before a peer is considered timedo ut.
    pub timeout_ms: u16,
    /// Duration of time (interval) between sending heartbeat messages.
    pub hb_dur_ms: u16,
    /// Set of pre-configured stream configurations
    pub stream: [Option<Stream>; super::MAX_NUM_STREAMS],
}

const fn arr_opt_from_slice<T: Copy, const N: usize>(slice: &[T]) -> [Option<T>; N] {
    let mut array = [const { None }; N];

    let mut count = 0;
    while count < slice.len() {
        array[count] = Some(slice[count]);
        count += 1;
    }

    array
}

crate::const_default!(
    Parameters => {
        timeout_ms: 3000,
        hb_dur_ms: 1000,
        id: Identity { sys: 1, com: 1 },
        stream: arr_opt_from_slice(&[
            Stream {
                message_id: Generator::ScaledImu as u32,
                interval_ms: 100,
            },
            Stream {
                message_id: Generator::ServoOutputRaw as u32,
                interval_ms: 100,
            },
            Stream {
                message_id: Generator::LocalPositionNed as u32,
                interval_ms: 20,
            },
            Stream {
                message_id: Generator::AttitudeQuaternion as u32,
                interval_ms: 50,
            },
        ])
    }
);

#[derive(mav_param::Tree, Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Identity {
    pub sys: u8,
    pub com: u8,
}

#[derive(Debug, Clone, Copy, mav_param::Tree, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stream {
    #[param(rename = "id")]
    pub message_id: u32,
    #[param(rename = "ms")]
    pub interval_ms: u16,
}

impl<V: MaybeVersioned> From<&Frame<V>> for Identity {
    fn from(frame: &Frame<V>) -> Self {
        Identity {
            sys: frame.system_id(),
            com: frame.component_id(),
        }
    }
}

impl Parameters {
    pub const fn timeout(&self) -> Duration {
        Duration::from_millis(self.timeout_ms as u64)
    }

    pub fn streams(&self) -> impl Iterator<Item = &Stream> {
        self.stream.iter().flatten()
    }
}

pub static TABLE: Table<Parameters> = Table::default("mav");
