//! SD card logging support for MicoAir H743v2.
//!
//! Hardware (SDMMC1, no external DMA -- H743 uses internal IDMA):
//!   CLK = PC12   CMD = PD2
//!   D0  = PC8    D1  = PC9    D2 = PC10   D3 = PC11
//!
//! # Architecture
//!
//! `SdmmcResources::setup()` allocates the `Sdmmc` driver in a `StaticCell` and
//! returns an `SdmmcDevice` that implements both `BlockDevice<512>` and `Reset`
//! (from `common::tasks::blackbox_fat`). Any code that needs SD logging can
//! pass this device to `blackbox_fat::main` (via `blackbox_task` below) or use
//! it directly with `embedded_fatfs`.
//!
//! # Reset strategy
//!
//! `SdmmcDevice` holds a `StorageDevice` created with `new_uninit_sd_card`.
//! Before any I/O, call `Reset::reset()` which calls `reacquire` to send CMD8,
//! ACMD41 and bring the card up.  Subsequent calls to `reset()` re-initialise
//! the card after a power cycle or error.
//!
//! # Partition offset detection
//!
//! Embassy's built-in `BlockDevice<512>` impl for `StorageDevice` does not add
//! a partition offset (see TODO in embassy sd.rs). `SdmmcDevice::reset()` reads
//! the MBR at sector 0 after the card is acquired. If bytes 510-511 are the
//! 0x55/0xAA signature, the partition start LBA is read from the first MBR
//! partition entry at bytes 454-457 (u32 LE) and stored in `boot_sector_offs`.
//! If no valid MBR is found the offset falls back to 0 (raw FAT volume).

use aligned::Aligned;
use common::{
    embedded_io::{ErrorKind, ErrorType},
    tasks::blackbox_fat::{BlockDevice, Reset},
};
use embassy_stm32::{
    bind_interrupts,
    peripherals,
    sdmmc::{
        self,
        sd::{Card, CmdBlock, DataBlock, StorageDevice},
        Error, Sdmmc,
    },
    time::Hertz,
    Peri,
};
// Peri is re-exported from embassy_hal_internal via embassy_stm32.
use static_cell::StaticCell;

// ── SdmmcDevice ──────────────────────────────────────────────────────────────

/// SDMMC1 block device for MicoAir H743v2.
pub struct SdmmcDevice {
    storage: StorageDevice<'static, 'static, Card>,
    freq: Hertz,
    /// Detected partition start LBA; 0 until `reset()` succeeds.
    boot_sector_offs: u32,
}

impl ErrorType for SdmmcDevice {
    type Error = ErrorKind;
}

impl SdmmcDevice {
    /// Like `Reset::reset()` but returns the underlying SDMMC error on failure,
    /// allowing callers to emit specific diagnostics (e.g. `NoCard` vs `Crc`).
    pub async fn try_reset(&mut self) -> Result<(), Error> {
        self.storage.reacquire(&mut CmdBlock::new(), self.freq).await?;

        // Read sector 0 and parse MBR to find the FAT partition start LBA.
        let mut block = DataBlock([0u32; 128]);
        self.boot_sector_offs = match self.storage.read_block(0, &mut block).await {
            Ok(()) => {
                let bytes: &[u8; 512] = unsafe { &*(&block.0 as *const [u32; 128] as *const [u8; 512]) };
                if bytes[510] == 0x55 && bytes[511] == 0xAA {
                    let lba = u32::from_le_bytes([bytes[454], bytes[455], bytes[456], bytes[457]]);
                    defmt::info!("sdlog: MBR ok, partition LBA={}", lba);
                    lba
                } else {
                    defmt::warn!("sdlog: no MBR signature, assuming raw FAT at sector 0");
                    0
                }
            }
            Err(_) => {
                defmt::warn!("sdlog: MBR read failed, assuming raw FAT at sector 0");
                0
            }
        };

        Ok(())
    }
}

impl Reset for SdmmcDevice {
    async fn reset(&mut self) -> bool {
        self.try_reset().await.is_ok()
    }
}

impl BlockDevice<512> for SdmmcDevice {
    type Error = Error;
    type Align = aligned::A4;

    async fn read(
        &mut self,
        block_address: u32,
        buf: &mut [Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Error> {
        self.storage.read(self.boot_sector_offs + block_address, buf).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        buf: &[Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Error> {
        self.storage.write(self.boot_sector_offs + block_address, buf).await
    }

    async fn size(&mut self) -> Result<u64, Error> {
        self.storage.size().await
    }
}

// ── SdmmcResources ───────────────────────────────────────────────────────────

/// SDMMC1 peripheral and pin resources.
///
/// Construct with the relevant fields from the embassy peripheral split, then
/// call `.setup()` to produce an `SdmmcDevice`.
pub struct SdmmcResources {
    pub periph: Peri<'static, peripherals::SDMMC1>,
    pub clk: Peri<'static, peripherals::PC12>,
    pub cmd: Peri<'static, peripherals::PD2>,
    pub d0: Peri<'static, peripherals::PC8>,
    pub d1: Peri<'static, peripherals::PC9>,
    pub d2: Peri<'static, peripherals::PC10>,
    pub d3: Peri<'static, peripherals::PC11>,
}

impl SdmmcResources {
    /// Allocate the SDMMC driver in a `StaticCell` and return an `SdmmcDevice`.
    ///
    /// Call `Reset::reset()` on the returned device before any I/O to
    /// initialise the SD card.
    pub fn setup(self) -> SdmmcDevice {
        bind_interrupts!(struct SdmmcIrq {
            SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
        });

        static SDMMC: StaticCell<Sdmmc<'static>> = StaticCell::new();

        let sdmmc = SDMMC.init(Sdmmc::new_4bit(
            self.periph,
            SdmmcIrq,
            self.clk,
            self.cmd,
            self.d0,
            self.d1,
            self.d2,
            self.d3,
            Default::default(),
        ));

        SdmmcDevice {
            storage: StorageDevice::new_uninit_sd_card(sdmmc),
            freq: Hertz::mhz(25),
            boot_sector_offs: 0,
        }
    }
}

// ── Blackbox task ─────────────────────────────────────────────────────────────

/// Embassy task: drains `BLACKBOX_QUEUE` and writes COBS+postcard records to
/// a sequentially-named `.LOG` file on the SD card.
///
/// Intended for the flight binary. Spawning this task is enough to enable full
/// blackbox logging; no other changes to `flight.rs` are required beyond
/// constructing `SdmmcResources` and passing the device here.
#[embassy_executor::task]
pub async fn blackbox_task(device: SdmmcDevice) -> ! {
    common::tasks::blackbox_fat::main::<_, { 512 * 64 }>(device).await
}
