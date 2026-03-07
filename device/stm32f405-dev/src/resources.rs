use core::ops::Range;

use assign_resources::assign_resources;

use common::{
    drivers::imu::ImuConfig,
    embassy_futures,
    embedded_io,
    errors::adapter::embedded_io::EmbeddedIoError,
    hw_abstraction::OutputGroup,
    serial::IoStreamRaw,
    types::config::{DshotConfig, I2cConfig, UartConfig, UartParity, UartStopBits},
};

use embassy_stm32::{
    bind_interrupts, exti::ExtiInput, mode::Async, peripherals, time::Hertz, Peri, Peripherals,
};
use static_cell::StaticCell;

assign_resources! {
    int_pin: IntPin {
        pin: PA10,
        exti: EXTI10,
    }
    i2c_1: I2c1 {
        periph: I2C1,
        sda: PB9,
        scl: PB8,
        tx_dma: DMA1_CH7,
        rx_dma: DMA1_CH0,
    }
    spi_1: Spi1 {
        periph: SPI1,
        sck: PB3,
        miso: PB4,
        mosi: PB5,
        rx_dma: DMA2_CH0,
        tx_dma: DMA2_CH5,
        cs_1: PC4,
        cs_2: PC5,
    }
    // usart_1: Usart1 {
    //     periph: USART1,
    //     rx_pin: PB7,
    //     tx_pin: PB6,
    //     rx_dma: DMA2_CH2,
    //     tx_dma: DMA2_CH7,
    // }
    // usart_2: Usart2 {
    //     periph: USART2,
    //     rx_pin: PA3,
    //     tx_pin: PA2,
    //     rx_dma: DMA1_CH5,
    //     tx_dma: DMA1_CH6,
    // }
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
        up_dma_1: DMA1_CH2,
        timer_1: TIM3,
        pin_1: PA6, //A2
        pin_2: PA7, //A3
        up_dma_2: DMA1_CH6,
        timer_2: TIM4,
        pin_3: PB6, //SCL
        pin_4: PB7, //SDA
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

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}

// ----------------------------------------------------------
// -------------------- Main IMU I2C ------------------------
// ----------------------------------------------------------

impl IntPin {
    pub fn _setup(self) -> impl embedded_hal_async::digital::Wait {
        ExtiInput::new(self.pin, self.exti, embassy_stm32::gpio::Pull::Up)
    }
}

impl I2c1 {
    pub fn setup(mut self, cfg: I2cConfig) -> impl embedded_hal_async::i2c::I2c {
        bind_interrupts!(struct I2c1Irq {
            I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C1>;
            I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C1>;
        });

        self.recover_bus();


        let mut config = embassy_stm32::i2c::Config::default();
        config.sda_pullup = cfg.sda_pullup;
        config.scl_pullup = cfg.scl_pullup;
        config.frequency = Hertz::hz(cfg.frequency);
        embassy_stm32::i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            I2c1Irq,
            self.tx_dma,
            self.rx_dma,
            config,
        )
    }

    fn recover_bus(&mut self) {
        use embassy_stm32::gpio::{Flex, Pull, Speed};

        let mut scl = Flex::new(self.scl.reborrow());
        let mut sda = Flex::new(self.sda.reborrow());

        scl.set_high();
        sda.set_high();
        scl.set_as_input_output_pull(Speed::Medium, Pull::Up);
        sda.set_as_input_output_pull(Speed::Medium, Pull::Up);

        // Clock out up to 16 pulses to release a slave holding SDA low.
        for _ in 0..16 {
            scl.set_low();
            short_delay();
            scl.set_high();
            short_delay();
        }

        // Force a STOP condition (SDA low->high while SCL is high).
        sda.set_low();
        short_delay();
        scl.set_high();
        short_delay();
        sda.set_high();
        short_delay();
    }
}

fn short_delay() {
    for _ in 0..128 {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::task]
pub(crate) async fn imu_reader(i2c: I2c1, i2c_cfg: I2cConfig, imu_cfg: ImuConfig) -> ! {
    let i2c = i2c.setup(i2c_cfg);
    common::tasks::imu_reader::main_6dof_i2c(i2c, imu_cfg, Some(0x69)).await
}

// ----------------------------------------------------------
// ------------------------- SPI ----------------------------
// ----------------------------------------------------------

impl Spi1 {
    pub fn setup(self) -> impl embedded_hal_async::spi::SpiBus {
        embassy_stm32::spi::Spi::new(
            self.periph,
            self.sck,
            self.mosi,
            self.miso,
            self.tx_dma,
            self.rx_dma,
            Default::default(),
        )
    }
}

#[embassy_executor::task]
pub(crate) async fn spi_reader(spi: Spi1) -> ! {
    let spi = spi.setup();
    todo!("Not implemented yet")
}

// ----------------------------------------------------------
// ---------------- Blackbox/SD logging ---------------------
// ----------------------------------------------------------

/// Thin wrapper around the Sdmmc peripheral so that we can store the frequency
/// alongside it, which is required when resetting the device.

#[cfg(feature = "sdmmc")]
pub(crate) mod sdmmc {
    use aligned::Aligned;
    use common::{
        embedded_io::{ErrorKind, ErrorType},
        tasks::blackbox_fat::{BlockDevice, Reset},
        types::config::SdmmcConfig,
    };
    use embassy_stm32::{
        bind_interrupts,
        sdmmc::{Error, Instance, InterruptHandler, Sdmmc},
        time::Hertz,
    };

    use crate::config::BLACKBOX_BUF;

    // NOTE: There is a TODO in the embassy `Sdmmc` read/write
    // impls about handling the partition begin offset. If the SD
    // card ever breaks when updating to newer versions of Embassy,
    // please check if this has already been implemented there.
    const BOOT_SECTOR_OFFS: u32 = 2048;

    pub struct SdmmcDevice<'d, T: Instance> {
        sdmmc: Sdmmc<'d, T>,
        frequency: Hertz,
    }

    impl<'d, T: Instance> ErrorType for SdmmcDevice<'d, T> {
        type Error = ErrorKind;
    }

    impl<'d, T: Instance> Reset for SdmmcDevice<'d, T> {
        async fn reset(&mut self) -> bool {
            self.sdmmc.init_sd_card(self.frequency).await.is_ok()
        }
    }

    impl<'d, T: Instance> BlockDevice<512> for SdmmcDevice<'d, T> {
        type Error = Error;
        type Align = aligned::A4;

        async fn read(
            &mut self,
            block_address: u32,
            data: &mut [Aligned<Self::Align, [u8; 512]>],
        ) -> Result<(), Self::Error> {
            // 2048 is the offset for the boot sector.
            self.sdmmc
                .read(BOOT_SECTOR_OFFS + block_address, data)
                .await
        }

        async fn write(
            &mut self,
            block_address: u32,
            data: &[Aligned<Self::Align, [u8; 512]>],
        ) -> Result<(), Self::Error> {
            // 2048 is the offset for the boot sector.
            self.sdmmc
                .write(BOOT_SECTOR_OFFS + block_address, data)
                .await
        }

        async fn size(&mut self) -> Result<u64, Self::Error> {
            self.sdmmc.size().await
        }
    }

    impl super::Sdcard {
        pub fn setup(self, config: SdmmcConfig) -> impl BlockDevice<512> + Reset {
            bind_interrupts!(struct SdioIrq {
                SDIO => InterruptHandler<super::peripherals::SDIO>;
            });

            SdmmcDevice {
                sdmmc: Sdmmc::new_4bit(
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
                frequency: Hertz::hz(config.frequency),
            }
        }
    }

    #[cfg(feature = "sdmmc")]
    #[embassy_executor::task]
    pub(crate) async fn blackbox_fat(device: super::Sdcard, config: SdmmcConfig) -> ! {
        let device = device.setup(config);
        common::tasks::blackbox_fat::main::<_, BLACKBOX_BUF>(device).await
    }
}

// ----------------------------------------------------------
// ---------------- FLASH config storage --------------------
// ----------------------------------------------------------

impl Flash {
    pub fn setup<'d>(self) -> impl common::embedded_storage_async::nor_flash::NorFlash {
        bind_interrupts!(struct FlashIrqs {
            FLASH => embassy_stm32::flash::InterruptHandler;
        });

        embassy_stm32::flash::Flash::new(self.periph, FlashIrqs)
    }
}

#[embassy_executor::task]
pub(crate) async fn param_storage(flash: Flash, range: Range<u32>) -> ! {
    let flash = flash.setup();
    common::tasks::param_storage::entry(flash, range).await
}

// ----------------------------------------------------------
// --------------- PWM-based motor driver -------------------
// ----------------------------------------------------------

impl MotorDriver {
    pub fn setup(&mut self, dshot: DshotConfig) -> impl OutputGroup + '_ {
        use crate::dshot_pwm::{DshotDriver2, UpDmaWaveform2};

        let first_pair = DshotDriver2::new(
            self.timer_1.reborrow(),
            self.pin_1.reborrow(),
            self.pin_2.reborrow(),
            UpDmaWaveform2::new(self.up_dma_1.reborrow()),
            dshot as u32,
        );

        let second_pair = DshotDriver2::new(
            self.timer_2.reborrow(),
            self.pin_3.reborrow(),
            self.pin_4.reborrow(),
            UpDmaWaveform2::new(self.up_dma_2.reborrow()),
            dshot as u32,
        );

        DualOutputGroup {
            first_pair,
            second_pair,
        }
    }
}

struct DualOutputGroup<A, B> {
    first_pair: A,
    second_pair: B,
}

impl<A, B> OutputGroup for DualOutputGroup<A, B>
where
    A: OutputGroup,
    B: OutputGroup,
{
    async fn set_motor_speeds(&mut self, speed: [u16; 4]) {
        embassy_futures::join::join(
            self.first_pair.set_motor_speeds([speed[0], speed[1], 0, 0]),
            self.second_pair.set_motor_speeds([speed[2], speed[3], 0, 0]),
        )
        .await;
    }

    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        embassy_futures::join::join(
            self.first_pair
                .set_reverse_dir([direction[0], direction[1], false, false]),
            self.second_pair
                .set_reverse_dir([direction[2], direction[3], false, false]),
        )
        .await;
    }

    async fn set_motor_speeds_min(&mut self) {
        embassy_futures::join::join(
            self.first_pair.set_motor_speeds_min(),
            self.second_pair.set_motor_speeds_min(),
        )
        .await;
    }

    async fn make_beep(&mut self) {}
}

#[embassy_executor::task]
pub(crate) async fn motor_governor(mut motors: MotorDriver, dshot_cfg: DshotConfig) -> ! {
    let motors = motors.setup(dshot_cfg);
    common::tasks::motor_governor::main(motors).await
}

#[embassy_executor::task]
pub(crate) async fn motor_direct_test(mut motors: MotorDriver, dshot_cfg: DshotConfig) -> ! {
    use common::embassy_time::Timer;

    defmt::info!("[motor-direct] waiting for ESC boot chime to complete");
    Timer::after_secs(10).await;

    let mut motors = motors.setup(dshot_cfg);

    defmt::info!("[motor-direct] starting raw DShot bench test");
    Timer::after_secs(2).await;

    for _ in 0..80 {
        motors.set_motor_speeds_min().await;
        Timer::after_millis(20).await;
    }

    for motor in 0..4usize {
        defmt::info!("[motor-direct] channel {} at fixed throttle", motor);

        let mut cmd = [0u16; 4];
        cmd[motor] = 900;

        for _ in 0..500 {
            motors.set_motor_speeds(cmd).await;
            Timer::after_millis(4).await;
        }

        for _ in 0..80 {
            motors.set_motor_speeds_min().await;
            Timer::after_millis(20).await;
        }
    }

    defmt::info!("[motor-direct] test complete, holding motors at minimum");
    loop {
        motors.set_motor_speeds_min().await;
        Timer::after_millis(20).await;
    }
}

#[embassy_executor::task]
pub(crate) async fn motor_pwm_test(mut motors: MotorDriver) -> ! {
    use common::embassy_time::Timer;
    use embassy_stm32::{
        gpio::OutputType,
        time::Hertz,
        timer::{
            low_level::CountingMode,
            simple_pwm::{PwmPin, SimplePwm},
            Channel,
        },
    };

    defmt::info!("[motor-pwm] waiting for ESC power-up");
    Timer::after_secs(4).await;

    let mut pwm_12 = SimplePwm::new(
        motors.timer_1.reborrow(),
        Some(PwmPin::new(motors.pin_1.reborrow(), OutputType::PushPull)),
        Some(PwmPin::new(motors.pin_2.reborrow(), OutputType::PushPull)),
        None,
        None,
        Hertz::hz(50),
        CountingMode::EdgeAlignedUp,
    );

    let mut pwm_34 = SimplePwm::new(
        motors.timer_2.reborrow(),
        Some(PwmPin::new(motors.pin_3.reborrow(), OutputType::PushPull)),
        Some(PwmPin::new(motors.pin_4.reborrow(), OutputType::PushPull)),
        None,
        None,
        Hertz::hz(50),
        CountingMode::EdgeAlignedUp,
    );

    pwm_12.channel(Channel::Ch1).enable();
    pwm_12.channel(Channel::Ch2).enable();
    pwm_34.channel(Channel::Ch1).enable();
    pwm_34.channel(Channel::Ch2).enable();

    let max_duty_12 = pwm_12.max_duty_cycle() as u32;
    let max_duty_34 = pwm_34.max_duty_cycle() as u32;

    let pulse_to_duty_12 = |pulse_us: u32| ((max_duty_12 * pulse_us) / 20_000) as u16;
    let pulse_to_duty_34 = |pulse_us: u32| ((max_duty_34 * pulse_us) / 20_000) as u16;

    let mut set_all = |pulse_us: u32| {
        pwm_12
            .channel(Channel::Ch1)
            .set_duty_cycle(pulse_to_duty_12(pulse_us));
        pwm_12
            .channel(Channel::Ch2)
            .set_duty_cycle(pulse_to_duty_12(pulse_us));
        pwm_34
            .channel(Channel::Ch1)
            .set_duty_cycle(pulse_to_duty_34(pulse_us));
        pwm_34
            .channel(Channel::Ch2)
            .set_duty_cycle(pulse_to_duty_34(pulse_us));
    };

    // Full ESC PWM calibration: max throttle -> minimum throttle
    defmt::info!("[motor-pwm] calibration step 1/2: full throttle");
    set_all(2000);
    Timer::after_secs(4).await;

    defmt::info!("[motor-pwm] calibration step 2/2: minimum throttle");
    set_all(1000);
    Timer::after_secs(6).await;

    defmt::info!("[motor-pwm] sweep all 4 motors: low -> medium");
    loop {
        set_all(1000);
        Timer::after_secs(2).await;

        for pulse_us in [1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500] {
            set_all(pulse_us);
            Timer::after_millis(800).await;
        }

        Timer::after_secs(2).await;
        set_all(1000);
        Timer::after_secs(2).await;
    }
}

// ----------------------------------------------------------
// --------------------- UARTs setup ------------------------
// ----------------------------------------------------------

#[allow(unused)]
pub struct UsartBuffered<'d> {
    pub rx: embassy_stm32::usart::RingBufferedUartRx<'d>,
    pub tx: embassy_stm32::usart::UartTx<'d, Async>,
}

macro_rules! impl_ring_buffered_usart_setup {
    ($fn_name:ident, $UsartX:ident, $USARTX:ident, rb = $rb_size:literal, $buf_rx:path, $buf_tx:path) => {
        #[allow(unused)]
        impl $UsartX {
            pub fn setup<'d>(&'d mut self, uart_cfg: UartConfig) -> UsartBuffered<'d> {
                bind_interrupts!(struct UsartIrq {
                    $USARTX => embassy_stm32::usart::InterruptHandler<peripherals::$USARTX>;
                });

                let mut config = embassy_stm32::usart::Config::default();
                config.baudrate = uart_cfg.baud;
                config.parity = match uart_cfg.parity {
                    UartParity::None => embassy_stm32::usart::Parity::ParityNone,
                    UartParity::Even => embassy_stm32::usart::Parity::ParityEven,
                    UartParity::Odd => embassy_stm32::usart::Parity::ParityOdd,
                };
                config.stop_bits = match uart_cfg.stop_bits {
                    UartStopBits::One => embassy_stm32::usart::StopBits::STOP1,
                    UartStopBits::Two => embassy_stm32::usart::StopBits::STOP2,
                };

                #[cfg(any(usart_v3, usart_v4))]
                {
                    config.invert_rx = uart_cfg.invert_rx;
                }

                #[cfg(not(any(usart_v3, usart_v4)))]
                if uart_cfg.invert_rx {
                    defmt::warn!("[uart] RX inversion requested but this USART peripheral does not support hardware inversion");
                }

                let usart = embassy_stm32::usart::Uart::new(
                    self.periph.reborrow(),
                    self.rx_pin.reborrow(),
                    self.tx_pin.reborrow(),
                    UsartIrq,
                    self.tx_dma.reborrow(),
                    self.rx_dma.reborrow(),
                    config,
                );

                let (tx, rx) = usart.unwrap().split();

                // Provide a static buffer for the ring buffer.
                use static_cell::StaticCell;
                static USART_BUFFER: StaticCell<[u8; $rb_size]> = StaticCell::new();
                let rx = rx.into_ring_buffered(USART_BUFFER.init([0; $rb_size]));

                UsartBuffered { rx, tx }
            }
        }

        #[embassy_executor::task]
        pub(crate) async fn $fn_name(mut usart: $UsartX, uart_cfg: UartConfig, serial_id: &'static str) {
            let usart = usart.setup(uart_cfg);

            let (rx, tx) = (usart.rx, usart.tx);

            let (mut dev_prod, app_cons) = $buf_tx.claim_reader();
            let (mut dev_cons, app_prod) = $buf_rx.claim_writer();

            let io_stream_raw = IoStreamRaw::new(serial_id, app_cons, app_prod);

            static IO_STREAM_RAW: StaticCell<IoStreamRaw<'static>> = StaticCell::new();
            let io_stream_ref = IO_STREAM_RAW.init(io_stream_raw);

            common::serial::insert(io_stream_ref).unwrap();

            let map_err = |error: embassy_stm32::usart::Error| {
                <embassy_stm32::usart::Error as embedded_io::Error>::kind(&error).into()
            };

            common::embassy_futures::join::join(
                dev_prod.embedded_io_connect_mapped(rx, map_err),
                dev_cons.embedded_io_connect_mapped(tx, map_err),
            ).await;

            defmt::warn!("[{}] Stream disconnected unexpectedly", serial_id)
        }
    };
}

use common::grantable_io::GrantableIo;

static BUF_RX1: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();
static BUF_TX1: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();

static BUF_RX2: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();
static BUF_TX2: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();

static BUF_RX3: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();
static BUF_TX3: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();

static BUF_RX6: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();
static BUF_TX6: GrantableIo<128, EmbeddedIoError> = GrantableIo::new();

// impl_ring_buffered_usart_setup!(run_usart1, Usart1, USART1, rb = 32, BUF_RX1, BUF_TX1);
// impl_ring_buffered_usart_setup!(run_usart2, Usart2, USART2, rb = 32, BUF_RX2, BUF_TX2);
impl_ring_buffered_usart_setup!(run_usart3, Usart3, USART3, rb = 32, BUF_RX3, BUF_TX3);
impl_ring_buffered_usart_setup!(run_usart6, Usart6, USART6, rb = 32, BUF_RX6, BUF_TX6);

// ----------------------------------------------------------
// --------------- USB for PC-FW connection -----------------
// ----------------------------------------------------------

#[cfg(feature = "usb")]
pub mod usb {
    use common::{embassy_usb::driver::Driver, types::device::HardwareInfo};
    use embassy_stm32::{
        bind_interrupts,
        usb::{Config, InterruptHandler},
    };

    // 2x USB packet size
    const USB_BUF_LEN: usize = 128;

    use crate::resources::Usb;
    use static_cell::StaticCell;

    impl Usb {
        pub fn setup(self) -> impl Driver<'static> {
            bind_interrupts!(pub struct UsbIrq {
                OTG_FS => InterruptHandler<embassy_stm32::peripherals::USB_OTG_FS>;
            });

            use embassy_stm32::interrupt::{self, InterruptExt, Priority};
            interrupt::OTG_FS.set_priority(Priority::P13);

            static USB_BUFFER: StaticCell<[u8; USB_BUF_LEN]> = StaticCell::new();
            let usb_buffer = USB_BUFFER.init([0u8; USB_BUF_LEN]);

            let mut config = Config::default();
            config.vbus_detection = false;

            embassy_stm32::usb::Driver::new_fs(
                self.usb,
                UsbIrq,
                self.dp,
                self.dm,
                usb_buffer,
                Default::default(),
            )
        }
    }

    #[embassy_executor::task]
    pub(crate) async fn runner(usb: Usb, info: HardwareInfo) -> ! {
        let usb = usb.setup();
        common::tasks::usb_manager::main(usb, info).await
    }
}

// ----------------------------------------------------------
// -------------------------- fin ---------------------------
// ----------------------------------------------------------


