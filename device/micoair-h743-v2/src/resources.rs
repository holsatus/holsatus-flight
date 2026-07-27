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
    },
    spi_2_extra: Spi2Extra {
        gyr_cs: PD5,
        acc_cs: PD4,
        gyr_dr: PC15,
        acc_dr: PC14,
        gyr_exti: EXTI15,
    },
    spi_3: Spi3 {
        periph: SPI3,
        sck: PB3,
        miso: PB4,
        mosi: PD6,
        rx_dma: DMA1_CH6,
        tx_dma: DMA1_CH7,
    },
    spi_3_extra: Spi3Extra {
        cs: PA15,
        dr: PB7,
        exti: EXTI7,
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
}

pub mod spi {
    use common::{
        drivers::{
            imu::{trigger::OnRising, Bmi088Config, Bmi088Spi, Bmi270Config, Bmi270Spi},
            wrapped::WrappedSpi,
        },
        embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex},
        embedded_hal_bus::spi::ExclusiveDevice,
        tasks::imu_reader::ImuRunner,
    };
    use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
    use embassy_stm32::exti::InterruptHandler;
    use embassy_stm32::interrupt::typelevel;
    use embassy_stm32::{
        bind_interrupts,
        exti::ExtiInput,
        gpio::{Level, Output, Pull, Speed},
    };

    stm32_support::impl_spi_setup!(super::Spi2: MODE_3, DMA1_STREAM4 => DMA1_CH4, DMA1_STREAM5 => DMA1_CH5);

    #[embassy_executor::task]
    pub(crate) async fn bmi088_reader(spi: super::Spi2, extra: super::Spi2Extra) -> ! {
        // The BMI088 is a strange beast; the acc and gyr as interfaced separately, so we need a shared SPI bus
        let spi = Mutex::<NoopRawMutex, _>::new(spi.setup());
        let acc_cs = Output::new(extra.acc_cs, Level::High, Speed::High);
        let acc_spi = SpiDevice::new(&spi, acc_cs);
        let gyr_cs = Output::new(extra.gyr_cs, Level::High, Speed::High);
        let gyr_spi = SpiDevice::new(&spi, gyr_cs);

        // Set up trigger to read IMU via the gyroscope data ready interrupt pin
        bind_interrupts!(struct Irqs { EXTI15_10 => InterruptHandler<typelevel::EXTI15_10>; });
        let int_pin = ExtiInput::new(extra.gyr_dr, extra.gyr_exti, Pull::Down, Irqs);

        let mut config = Bmi088Config::default();
        config.pin_3_int_data_ready = true;

        ImuRunner::entry::<(Bmi088Spi, _, _)>(
            (WrappedSpi(acc_spi), WrappedSpi(gyr_spi)),
            config,
            OnRising(int_pin),
        )
        .await
    }

    stm32_support::impl_spi_setup!(super::Spi3: MODE_3, DMA1_STREAM6 => DMA1_CH6, DMA1_STREAM7 => DMA1_CH7);

    #[embassy_executor::task]
    pub(crate) async fn bmi270_reader(spi: super::Spi3, extra: super::Spi3Extra) -> ! {
        let spi = spi.setup();

        let cs = Output::new(extra.cs, Level::High, Speed::High);
        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        // Set up trigger to read IMU via the data ready interrupt pin
        bind_interrupts!(struct Irqs { EXTI9_5 => InterruptHandler<typelevel::EXTI9_5>; });
        let int_pin = ExtiInput::new(extra.dr, extra.exti, Pull::Down, Irqs);

        let mut config = Bmi270Config::default();
        config.pin_1_int_data_ready = true;

        ImuRunner::entry::<(Bmi270Spi, _)>(WrappedSpi(spi_device), config, OnRising(int_pin)).await
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
