use embassy_time::Duration;
use mavio::{prelude::MaybeVersioned, protocol::Frame};

use crate::{mavlink2::messages::Generator, tasks::param_storage::Table};

#[derive(mav_param::Tree, Default, Clone)]
pub struct Parameters {
    pub id: PeerId,
    pub timeout_ms: u16,
    pub hb_dur_ms: u16,
    pub stream: [Option<Stream>; 8],
}

#[derive(mav_param::Tree, Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PeerId {
    pub sys: u8,
    pub com: u8,
}

#[derive(Debug, Clone, mav_param::Tree, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Stream {
    #[param(rename = "id")]
    message_id: u32,
    #[param(rename = "ms")]
    interval_ms: u16,
}

impl<V: MaybeVersioned> From<&Frame<V>> for PeerId {
    fn from(frame: &Frame<V>) -> Self {
        PeerId {
            sys: frame.system_id(),
            com: frame.component_id(),
        }
    }
}

impl Parameters {
    pub const fn const_default() -> Parameters {
        Parameters {
            timeout_ms: 3000,
            hb_dur_ms: 500,
            id: PeerId { sys: 1, com: 1 },
            stream: [
                Some(Stream {
                    message_id: Generator::Heartbeat as u32,
                    interval_ms: 1000,
                }),
                Some(Stream {
                    message_id: Generator::ScaledImu as u32,
                    interval_ms: 100,
                }),
                Some(Stream {
                    message_id: Generator::ServoOutputRaw as u32,
                    interval_ms: 100,
                }),
                None,
                None,
                None,
                None,
                None,
            ],
        }
    }

    pub const fn timeout(&self) -> Duration {
        Duration::from_millis(self.timeout_ms as u64)
    }

    pub fn streams(&self) -> impl Iterator<Item = &Stream> {
        self.stream.iter().filter_map(|e| e.as_ref())
    }
}

pub static TABLE: Table<Parameters> = Table::new("mav", Parameters::const_default());
