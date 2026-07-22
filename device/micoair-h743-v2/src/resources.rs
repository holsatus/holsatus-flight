use assign_resources::assign_resources;
use embassy_stm32::{peripherals, Peri, Peripherals};

assign_resources! {
    i2c_2: I2c2 {
        periph: I2C2,
        sda: PB11,
        scl: PB10,
        rx_dma: DMA2_CH4,
        tx_dma: DMA2_CH5,
        baro_dr: PD0,
    },
    spi_2: Spi2 {
        periph: SPI2,
        sck: PD3,
        miso: PC2,
        mosi: PC3,
        rx_dma: DMA1_CH4,
        tx_dma: DMA1_CH5,
        gyr_cs: PD5,
        acc_cs: PD4,
        gyr_dr: PC15,
        acc_dr: PC14,
    },
    spi_3: Spi3 {
        periph: SPI3,
        sck: PB3,
        miso: PB4,
        mosi: PD6,
        rx_dma: DMA1_CH6,
        tx_dma: DMA1_CH7,
        cs: PA15,
        dr: PB7,
    },
    usart_1: Usart1 {
        periph: USART1,
        rx_pin: PB15,
        tx_pin: PB14,
        rx_dma: DMA2_CH6,
        tx_dma: DMA2_CH7,
    },
    usart_2: Usart2 {
        periph: USART2,
        rx_pin: PA3,
        tx_pin: PA2,
        rx_dma: DMA1_CH2,
        tx_dma: DMA1_CH3,
    },
    usart_3: Usart3 {
        periph: USART3,
        rx_pin: PD9,
        tx_pin: PD8,
        rx_dma: DMA1_CH0,
        tx_dma: DMA2_CH2,
    },
    usart_6: Usart6 {
        periph: USART6,
        rx_pin: PC7,
        tx_pin: PC6,
        rx_dma: DMA2_CH0,
        tx_dma: DMA2_CH1,
    },
    motors: MotorDriver {
        timer: TIM1,
        up_dma: DMA1_CH1,
        pin_1: PE9,
        pin_2: PE11,
        pin_3: PE13,
        pin_4: PE14,
    },
    sdcard: Sdcard {
        periph: SDMMC1,
        clk: PC12,
        cmd: PD2,
        d0: PC8,
        d1: PC9,
        d2: PC10,
        d3: PC11,
    },
    flash: Flash {
        periph: FLASH
    },
    usb: Usb {
        usb: USB_OTG_FS,
        dp: PA12,
        dm: PA11,
    },
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}

pub mod i2c {
    // Usage would be similar:
    stm32_support::impl_i2c_setup!(
        super::I2c2,
        I2C2_EV => I2C2,
        I2C2_ER => I2C2,
        DMA2_STREAM4 => DMA2_CH4,
        DMA2_STREAM5 => DMA2_CH5,
    );

    #[embassy_executor::task]
    pub(crate) async fn imu_reader(
        i2c: super::I2c2,
        i2c_cfg: common::types::config::I2cConfig,
        imu_cfg: common::drivers::imu::ImuConfig,
    ) -> ! {
        let i2c = i2c.setup(i2c_cfg);
        common::tasks::imu_reader::main_6dof_i2c(i2c, imu_cfg, Some(0x69)).await
    }
}

pub mod spi {
    stm32_support::impl_spi_setup!(super::Spi2, DMA1_STREAM4 => DMA1_CH4, DMA1_STREAM5 => DMA1_CH5);

    #[embassy_executor::task]
    pub(crate) async fn spi_2_imu_reader(spi: super::Spi2) -> ! {
        let _spi = spi.setup();
        // common::tasks::imu_reader::main_6dof_spi(_spi, imu_cfg).await
        todo!("Not implemented yet")
    }

    stm32_support::impl_spi_setup!(super::Spi3, DMA1_STREAM6 => DMA1_CH6, DMA1_STREAM7 => DMA1_CH7);

    #[embassy_executor::task]
    pub(crate) async fn spi_3_imu_reader(spi: super::Spi3) -> ! {
        let _spi = spi.setup();
        // common::tasks::imu_reader::main_6dof_spi(_spi, imu_cfg).await
        todo!("Not implemented yet")
    }
}

#[cfg(feature = "sdmmc")]
pub(crate) mod sdmmc {
    use common::types::config::SdmmcConfig;

    stm32_support::impl_sdmmc_setup!(super::Sdcard, SDMMC1 => SDMMC1);

    #[embassy_executor::task]
    pub(crate) async fn blackbox_fat(device: super::Sdcard, config: SdmmcConfig) -> ! {
        let device = device.setup(config);
        common::tasks::blackbox_fat::main::<_, { 512 * 2 }>(device).await
    }
}

pub mod flash {
    stm32_support::impl_flash_setup!(super::Flash, FLASH);

    #[embassy_executor::task]
    pub(crate) async fn param_storage(flash: super::Flash, range: core::ops::Range<u32>) -> ! {
        let flash = flash.setup();
        common::tasks::param_storage::entry(flash, range).await
    }
}

pub mod motors {
    stm32_support::impl_up_dma_dshot_setup!(super::MotorDriver, DMA1_STREAM1 => DMA1_CH1);

    #[embassy_executor::task]
    pub(crate) async fn motor_governor(
        mut motors: super::MotorDriver,
        dshot_cfg: common::types::config::DshotConfig,
    ) -> ! {
        let motors = motors.setup(dshot_cfg);
        common::tasks::motor_governor::main(motors).await
    }
}

pub mod usart {
    stm32_support::impl_usart_setup!(
        super::Usart1,
        USART1 => USART1,
        DMA2_STREAM6 => DMA2_CH6,
        DMA2_STREAM7 => DMA2_CH7,
        runner = run_usart1,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart2,
        USART2 => USART2,
        DMA1_STREAM2 => DMA1_CH2,
        DMA1_STREAM3 => DMA1_CH3,
        runner = run_usart2,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart3,
        USART3 => USART3,
        DMA1_STREAM0 => DMA1_CH0,
        DMA2_STREAM2 => DMA2_CH2,
        runner = run_usart3,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart6,
        USART6 => USART6,
        DMA2_STREAM0 => DMA2_CH0,
        DMA2_STREAM1 => DMA2_CH1,
        runner = run_usart6,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );
}

#[cfg(feature = "usb")]
pub mod usb {
    use crate::resources::Usb;
    use common::types::device::HardwareInfo;

    stm32_support::impl_usb_setup!(Usb, OTG_FS => USB_OTG_FS, 128);

    #[embassy_executor::task]
    pub(crate) async fn runner(usb: Usb, info: HardwareInfo) -> ! {
        let usb = usb.setup();
        common::tasks::usb_manager::main(usb, info).await
    }
}
