use core::ops::Range;

use aligned::Aligned;
use assign_resources::assign_resources;
use common::{
    drivers::imu::ImuConfig, embassy_usb::driver::Driver, embedded_io, hw_abstraction::FourMotors, tasks::blackbox_fat::{BlockDevice, Reset}, types::{config::{DshotConfig, I2cConfig, SdmmcConfig, UartConfig}, device::HardwareInfo}
};
use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, mode::Async, peripherals, sdmmc::{Error, Instance, Sdmmc, SdmmcDma}, time::Hertz
};

use crate::config::{gen_default_cfg, BLACKBOX_BUF};

assign_resources! {
    int_pin: IntPin {
        pin: PA10,
        exti: EXTI10,
    }
    i2c_1: I2c1 {
        periph: I2C1,
        sda: PB9,
        scl: PB8,
        tx_dma: DMA1_CH6,
        rx_dma: DMA1_CH0,
    }
    usart_1: Usart1 {
        periph: USART1,
        rx_pin: PB7,
        tx_pin: PA9,
        rx_dma: DMA2_CH2,
        tx_dma: DMA2_CH7,
    }
    usart_3: Usart3 {
        periph: USART3,
        rx_pin: PB11,
        tx_pin: PB10,
        rx_dma: DMA1_CH1,
        tx_dma: DMA1_CH3,
    }
    usart_6: Usart6 {
        periph: USART6,
        rx_pin: PC7,
        tx_pin: PC6,
        rx_dma: DMA2_CH1,
        tx_dma: DMA2_CH6,
    }
    motors: MotorDriver {
        timer: TIM3,
        pin_1: PA6,
        pin_2: PA7,
        pin_3: PB0,
        pin_4: PB1,
        dma_1: DMA1_CH4,
        dma_2: DMA1_CH5,
        dma_3: DMA1_CH7,
        dma_4: DMA1_CH2,
    }
    sdcard: Sdcard {
        periph: SDIO,
        dma: DMA2_CH3,
        clk: PC12,
        cmd: PD2,
        d0: PC8,
        d1: PC9,
        d2: PC10,
        d3: PC11,
    }
    flash: Flash {
        periph: FLASH
    }
    usb: Usb {
        usb: USB_OTG_FS,
        dp: PA12,
        dm: PA11,
    }
}

// ----------------------------------------------------------
// -------------------- Main IMU I2C ------------------------
// ----------------------------------------------------------

impl IntPin {
    pub fn setup(self) -> impl embedded_hal_async::digital::Wait {
        ExtiInput::new(self.pin, self.exti, embassy_stm32::gpio::Pull::Up)
    }
}

impl I2c1 {
    pub fn setup(self, cfg: &I2cConfig) -> impl embedded_hal_async::i2c::I2c {
        bind_interrupts!(struct I2c1Irq {
            I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
            I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
        });

        let mut config = embassy_stm32::i2c::Config::default();
        config.sda_pullup = cfg.sda_pullup;
        config.scl_pullup = cfg.scl_pullup;
        embassy_stm32::i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            I2c1Irq,
            self.tx_dma,
            self.rx_dma,
            Hertz::hz(cfg.frequency),
            config,
        )
    }
}

#[embassy_executor::task]
pub(crate) async fn imu_reader_6dof(i2c: I2c1, pin: IntPin, i2c_cfg: &'static I2cConfig, imu_cfg: &'static ImuConfig) -> ! {
    let i2c = i2c.setup(i2c_cfg);
    let _pin = pin.setup();
    common::tasks::imu_reader::main_6dof_i2c(i2c, imu_cfg, Some(0x69)).await
}

// ----------------------------------------------------------
// ---------------- Blackbox/SD logging ---------------------
// ----------------------------------------------------------

/// Thin wrapper around the Sdmmc peripheral so that we can store the frequency
/// alongside it, which is required when resetting the device.
pub struct SdmmcDevice<'d, T: Instance, D: SdmmcDma<T>> {
    sdmmc: Sdmmc<'d, T, D>,
    frequency: Hertz,
}

impl<'d, T: Instance, D: SdmmcDma<T>> embedded_io::ErrorType for SdmmcDevice<'d, T, D> {
    type Error = embedded_io::ErrorKind;
}

impl<'d, T: Instance, D: SdmmcDma<T>> Reset for SdmmcDevice<'d, T, D> {
    async fn reset(&mut self) -> bool {
        self.sdmmc.init_card(self.frequency).await.is_ok()
    }
}

impl<'d, T: Instance, D: SdmmcDma<T>> BlockDevice<512> for SdmmcDevice<'d, T, D> {
    type Error = Error;

    type Align = aligned::A4;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        // 2048 is the offset for the boot sector.
        self.sdmmc.read(block_address + 2048, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        // 2048 is the offset for the boot sector.
        self.sdmmc.write(block_address + 2048, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.sdmmc.size().await
    }
}

impl Sdcard {
    pub fn setup(self, config: &SdmmcConfig) -> impl BlockDevice<512> + Reset {
        bind_interrupts!(struct SdioIrq {
            SDIO => embassy_stm32::sdmmc::InterruptHandler<peripherals::SDIO>;
        });

        SdmmcDevice {
            sdmmc: embassy_stm32::sdmmc::Sdmmc::new_4bit(
                self.periph,
                SdioIrq,
                self.dma,
                self.clk,
                self.cmd,
                self.d0,
                self.d1,
                self.d2,
                self.d3,
                Default::default(),
            ),
            frequency: Hertz::hz(config.frequency), // TODO make this configurable
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn blackbox_fat(device: Sdcard, config: &'static SdmmcConfig) -> ! {
    let device = device.setup(config);
    common::tasks::blackbox_fat::main::<_, BLACKBOX_BUF>(device).await
}

// ----------------------------------------------------------
// ---------------- FLASH config storage --------------------
// ----------------------------------------------------------

impl Flash {
    pub fn setup<'d>(self) -> (impl common::embedded_storage_async::nor_flash::NorFlash, Range<u32>) {
        bind_interrupts!(struct FlashIrqs {
            FLASH => embassy_stm32::flash::InterruptHandler;
        });

        let flash = embassy_stm32::flash::Flash::new(self.periph, FlashIrqs);
        let range = 0x80000..0x100000;
        (flash, range)
        
    }
}

#[embassy_executor::task]
pub(crate) async fn configurator(flash: Flash) -> ! {
    let (flash, range) = flash.setup();
    common::tasks::configurator::main(flash, range, gen_default_cfg).await
}

// ----------------------------------------------------------
// --------------- PWM-based motor driver -------------------
// ----------------------------------------------------------

impl MotorDriver {
    pub fn setup(&mut self, dshot: &DshotConfig) -> impl FourMotors + '_ {
        crate::dshot_pwm::DshotPwm::new(
            &mut self.timer,
            &mut self.pin_1,
            &mut self.pin_2,
            &mut self.pin_3,
            &mut self.pin_4,
            &mut self.dma_1,
            &mut self.dma_2,
            &mut self.dma_3,
            &mut self.dma_4,
            *dshot as u32,
        )
    }
}

#[embassy_executor::task]
pub(crate) async fn motor_governor(mut motors: MotorDriver, dshot_cfg: &'static DshotConfig) -> ! {
    loop {
        let motors = motors.setup(dshot_cfg);
        common::tasks::motor_governor::main(motors).await
    }
}

// ----------------------------------------------------------
// --------------------- UARTs setup ------------------------
// ----------------------------------------------------------

pub struct UsartBuffered<'d> {
    pub rx: embassy_stm32::usart::RingBufferedUartRx<'d>,
    pub tx: embassy_stm32::usart::UartTx<'d, Async>,
}

macro_rules! impl_ring_buffered_usart_setup {
    ($UsartX:ident, $USARTX:ident, $size:literal) => {
        #[allow(unused)]
        impl $UsartX {
            pub fn setup<'d>(&'d mut self, uart_cfg: &UartConfig) -> UsartBuffered<'d> {
                bind_interrupts!(struct UsartIrq {
                    $USARTX => embassy_stm32::usart::InterruptHandler<peripherals::$USARTX>;
                });

                let mut config = embassy_stm32::usart::Config::default();
                config.baudrate = uart_cfg.baud;

                let usart = embassy_stm32::usart::Uart::new(
                    &mut self.periph,
                    &mut self.rx_pin,
                    &mut self.tx_pin,
                    UsartIrq,
                    &mut self.tx_dma,
                    &mut self.rx_dma,
                    config,
                );

                let (tx, rx) = usart.unwrap().split();

                // Provide a static buffer for the ring buffer.
                use static_cell::StaticCell;
                static USART_BUFFER: StaticCell<[u8; $size]> = StaticCell::new();
                let rx = rx.into_ring_buffered(USART_BUFFER.init([0; $size]));

                UsartBuffered { rx, tx }
            }

            pub fn any(self) -> AnyUsart {
                AnyUsart::$UsartX(self)
            }
        }
    };
}

impl_ring_buffered_usart_setup!(Usart1, USART1, 128);
impl_ring_buffered_usart_setup!(Usart3, USART3, 128);
impl_ring_buffered_usart_setup!(Usart6, USART6, 128);

/// Type-erased enum for any USART peripheral
#[allow(unused)]
pub enum AnyUsart {
    Usart1(Usart1),
    Usart3(Usart3),
    Usart6(Usart6),
}

impl AnyUsart {
    fn setup(&mut self, cfg: &UartConfig) -> UsartBuffered<'_> {
        match self {
            AnyUsart::Usart1(uart) => uart.setup(cfg),
            AnyUsart::Usart3(uart) => uart.setup(cfg),
            AnyUsart::Usart6(uart) => uart.setup(cfg),
        }
    }
}

// RC controls (e.g. sbus/crsf)
#[embassy_executor::task]
pub(crate) async fn rc_serial_read(mut usart: AnyUsart, uart_cfg: &'static UartConfig) -> ! {
    loop {
        let usart = usart.setup(uart_cfg);
        common::tasks::rc_reader::main(usart.rx).await
    }
}

// GNSS module (ubx protocol)
#[embassy_executor::task]
pub(crate) async fn gnss_serial_read(mut usart: AnyUsart, uart_cfg: &'static UartConfig) -> ! {
    loop {
        let usart = usart.setup(uart_cfg);
        common::tasks::gnss_reader::main(usart.rx).await
    }
}

// Mavlink telemetry
#[embassy_executor::task]
pub(crate) async fn mavlink_serial(mut usart: AnyUsart, uart_cfg: &'static UartConfig) -> ! {
    let server = common::mavlink::MavServer::new();
    loop {
        let mut usart = usart.setup(uart_cfg);
        server.main(&mut usart.rx, &mut usart.tx).await
    }
}

// ----------------------------------------------------------
// --------------- USB for PC-FW connection -----------------
// ----------------------------------------------------------

impl Usb {
    pub fn setup(self) -> impl Driver<'static> {
        bind_interrupts!(pub struct UsbIrq {
            OTG_FS => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
        });

        use static_cell::StaticCell;
        const USB_BUF_LEN: usize = 256;
        static USB_BUFFER: StaticCell<[u8; USB_BUF_LEN]> = StaticCell::new();
        let usb_buffer = USB_BUFFER.init([0u8; USB_BUF_LEN]);

        let mut config = embassy_stm32::usb::Config::default();
        config.vbus_detection = false;

        embassy_stm32::usb::Driver::new_fs(self.usb, UsbIrq, self.dp, self.dm, usb_buffer, config)
    }
}

#[embassy_executor::task]
pub(crate) async fn usb_manager(usb: Usb, info: &'static HardwareInfo) -> ! {
    let usb = usb.setup();
    common::tasks::usb_manager::main(usb, info).await
}

// ----------------------------------------------------------
// -------------------------- fin ---------------------------
// ----------------------------------------------------------
