use assign_resources::assign_resources;
use embassy_stm32::{peripherals, Peri, Peripherals};

assign_resources! {
    int_pin: IntPin {
        pin: PA10,
        exti: EXTI10,
    },
    i2c_1: I2c1 {
        periph: I2C1,
        sda: PB9,
        scl: PB8,
        tx_dma: DMA1_CH7,
        rx_dma: DMA1_CH0,
    },
    spi_1: Spi1 {
        periph: SPI1,
        sck: PB3,
        miso: PB4,
        mosi: PB5,
        rx_dma: DMA2_CH0,
        tx_dma: DMA2_CH5,
        cs_1: PA0,
        cs_2: PA1,
    },
    usart_1: Usart1 {
        periph: USART1,
        rx_pin: PB7,
        tx_pin: PA9,
        rx_dma: DMA2_CH2,
        tx_dma: DMA2_CH7,
    },
    usart_2: Usart2 {
        periph: USART2,
        rx_pin: PA3,
        tx_pin: PA2,
        rx_dma: DMA1_CH5,
        tx_dma: DMA1_CH6,
    },
    usart_3: Usart3 {
        periph: USART3,
        rx_pin: PB11,
        tx_pin: PB10,
        rx_dma: DMA1_CH1,
        tx_dma: DMA1_CH3,
    },
    usart_6: Usart6 {
        periph: USART6,
        rx_pin: PC7,
        tx_pin: PC6,
        rx_dma: DMA2_CH1,
        tx_dma: DMA2_CH6,
    },
    motors: MotorDriver {
        timer: TIM3,
        up_dma: DMA1_CH2,
        pin_1: PA6,
        pin_2: PA7,
        pin_3: PB0,
        pin_4: PB1,
    },
    sdcard: Sdcard {
        periph: SDIO,
        dma: DMA2_CH3,
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
    use common::{
        drivers::{imu::icm20948::*, wrapped::WrappedI2c},
        embassy_time::{Duration, Ticker},
    };

    stm32_support::impl_i2c_setup!(
        super::I2c1,
        I2C1_EV => I2C1,
        I2C1_ER => I2C1,
        DMA1_STREAM0 => DMA1_CH0,
        DMA1_STREAM7 => DMA1_CH7,
    );

    #[embassy_executor::task]
    pub(crate) async fn imu_reader(
        i2c: super::I2c1,
        i2c_cfg: common::types::config::I2cConfig,
        imu_cfg: Config,
    ) -> ! {
        let i2c = i2c.setup(i2c_cfg);

        common::tasks::imu_reader::ImuRunner::entry::<(Icm209486DofI2c, _)>(
            WrappedI2c(i2c),
            (0x69, imu_cfg),
            Ticker::every(Duration::from_hz(1125)),
        )
        .await
    }
}

pub mod spi {
    stm32_support::impl_spi_setup!(super::Spi1: MODE_3, DMA2_STREAM5 => DMA2_CH5, DMA2_STREAM0 => DMA2_CH0);

    #[embassy_executor::task]
    pub(crate) async fn spi_reader(spi: super::Spi1) -> ! {
        let _spi = spi.setup();
        todo!("Not implemented yet")
    }
}

#[cfg(feature = "sdmmc")]
pub(crate) mod sdmmc {
    use common::types::config::SdmmcConfig;

    stm32_support::impl_sdmmc_setup!(super::Sdcard, SDIO => SDIO, DMA2_STREAM3 => DMA2_CH3);

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
    stm32_support::impl_up_dma_dshot_setup!(super::MotorDriver, DMA1_STREAM2 => DMA1_CH2);

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
        DMA2_STREAM2 => DMA2_CH2,
        DMA2_STREAM7 => DMA2_CH7,
        runner = run_usart1,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart2,
        USART2 => USART2,
        DMA1_STREAM5 => DMA1_CH5,
        DMA1_STREAM6 =>DMA1_CH6,
        runner = run_usart2,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart3,
        USART3 => USART3,
        DMA1_STREAM1 => DMA1_CH1,
        DMA1_STREAM3 => DMA1_CH3,
        runner = run_usart3,
        ring_buf = 32,
        rx_buf = 256,
        tx_buf = 512,
    );

    stm32_support::impl_usart_setup!(
        super::Usart6,
        USART6 => USART6,
        DMA2_STREAM1 => DMA2_CH1,
        DMA2_STREAM6 => DMA2_CH6,
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
