use aligned::Aligned;
use common::{
    embedded_io::{ErrorKind, ErrorType},
    tasks::blackbox_fat::{BlockDevice, Reset},
};
use embassy_stm32::{
    sdmmc::{
        sd::{Card, CmdBlock, StorageDevice},
        Error,
    },
    time::Hertz,
};

// NOTE: There is a TODO in the embassy `Sdmmc` read/write
// impls about handling the partition begin offset. If the SD
// card ever breaks when updating to newer versions of Embassy,
// please check if this has already been implemented there.
const BOOT_SECTOR_OFFS: u32 = 2048;

pub struct SdmmcDevice<'d> {
    pub storage: StorageDevice<'d, 'd, Card>,
    pub freq: Hertz,
}

impl ErrorType for SdmmcDevice<'_> {
    type Error = ErrorKind;
}

impl Reset for SdmmcDevice<'_> {
    async fn reset(&mut self) -> bool {
        self.storage
            .reacquire(&mut CmdBlock::new(), self.freq)
            .await
            .is_ok()
    }
}

impl BlockDevice<512> for SdmmcDevice<'_> {
    type Error = Error;
    type Align = aligned::A4;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        self.storage
            .read(BOOT_SECTOR_OFFS + block_address, data)
            .await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        self.storage
            .write(BOOT_SECTOR_OFFS + block_address, data)
            .await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.storage.size().await
    }
}
