
#[cfg(feature = "rp2040")]
pub mod rp2040;
#[cfg(feature = "rp2040")]
pub use rp2040 as bsp;

/// The board info structure
struct BoardInfo
{
    pub firmware_name: & 'static str,
    pub firmware_version: & 'static str,
    pub firmware_url: & 'static str,
    pub machine_board: & 'static str,
    pub machine_processor: & 'static str,
}



impl BoardInfo {
    #[allow(unused)]
    pub(crate) const fn new() -> Self {
        Self {
            firmware_name: "Holsatus Flight",
            firmware_version: env!("CARGO_PKG_VERSION"),
            firmware_url: "https://github.com/holsatus/holsatus-flight",
            machine_board: bsp::MACHINE_BOARD,
            machine_processor: bsp::MACHINE_PROCESSOR,
        }
    }
}

pub static MACHINE_INFO: BoardInfo = BoardInfo::new();