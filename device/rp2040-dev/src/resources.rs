use common::{
    drivers::imu::ImuConfig,
    embedded_io,
    errors::adapter::embedded_io::EmbeddedIoError,
    hw_abstraction::OutputGroup,
    serial::IoStreamRaw,
    types::config::{DshotConfig, I2cConfig, UartConfig},
};

use embassy_rp::{bind_interrupts, peripherals, Peri, Peripherals};

/// Define the clock configuration for the board.
pub(crate) fn config() -> embassy_rp::config::Config {
    let mut config = embassy_rp::config::Config::default();

    if let Some(xosc) = config.clocks.xosc.as_mut() {
        if let Some(sys_pll) = xosc.sys_pll.as_mut() {
            sys_pll.post_div2 = 1;
        }
    }
    config
}

assign_resources::assign_resources! {
    i2c_0: I2c0 {
        periph: I2C0,
        sda: PIN_16,
        scl: PIN_17,
        dma: DMA_CH1,
    }
    uart_0: Uart0 {
        periph: UART0,
        tx_pin: PIN_0,
        rx_pin: PIN_1,
        tx_dma: DMA_CH2,
        rx_dma: DMA_CH3,
    }
    uart_1: Uart1 {
        periph: UART1,
        tx_pin: PIN_8,
        rx_pin: PIN_9,
        tx_dma: DMA_CH4,
        rx_dma: DMA_CH5,
    }
    motors: MotorDriver {
        pio: PIO0,
        pin_1: PIN_10,
        pin_2: PIN_11,
        pin_3: PIN_12,
        pin_4: PIN_13,
    }
    flash: Flash {
        periph: FLASH,
        dma: DMA_CH0,
    }
    usb: Usb {
        usb: USB,
    }
    core1: Core1 {
        core: CORE1,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}

// ----------------------------------------------------------
// -------------------- Main IMU I2C ------------------------
// ----------------------------------------------------------

impl I2c0 {
    pub fn setup(self, cfg: I2cConfig) -> impl common::embedded_hal_async::i2c::I2c {
        bind_interrupts!(struct I2c0Irq {
            I2C0_IRQ => embassy_rp::i2c::InterruptHandler<peripherals::I2C0>;
        });

        let mut config = embassy_rp::i2c::Config::default();
        config.frequency = cfg.frequency;
        embassy_rp::i2c::I2c::new_async(self.periph, self.scl, self.sda, I2c0Irq, config)
    }
}

#[embassy_executor::task]
pub(crate) async fn imu_reader(i2c: I2c0, i2c_cfg: I2cConfig, imu_cfg: ImuConfig) -> ! {
    let i2c = i2c.setup(i2c_cfg);
    common::tasks::imu_reader::main_6dof_i2c(i2c, imu_cfg, Some(0x69)).await
}

// ----------------------------------------------------------
// ---------------- FLASH config storage --------------------
// ----------------------------------------------------------

impl Flash {
    pub fn setup<'d>(self) -> impl common::embedded_storage_async::nor_flash::NorFlash {
        bind_interrupts!(struct Irqs {
            DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<peripherals::DMA_CH0>;
        });

        embassy_rp::flash::Flash::<_, _, { 2 * 1024 * 1024 }>::new(self.periph, self.dma, Irqs)
    }
}

#[embassy_executor::task]
pub(crate) async fn param_storage(flash: Flash) -> ! {
    let flash = flash.setup();
    common::tasks::param_storage::entry(flash, 0..{ 2 * 1024 * 1024 }).await
}

// ----------------------------------------------------------
// --------------- PWM-based motor driver -------------------
// ----------------------------------------------------------

struct PioMotors<'a, PIO: embassy_rp::pio::Instance> {
    inner: crate::dshot_pio::DshotPio<'a, 4, PIO>,
}

impl<'a, PIO: embassy_rp::pio::Instance> OutputGroup for PioMotors<'a, PIO> {
    async fn set_motor_speeds(&mut self, speeds: [u16; 4]) {
        self.inner
            .command(speeds.map(|speed| dshot_encoder::throttle_clamp(speed, false)));
    }

    async fn set_motor_speeds_min(&mut self) {
        self.inner
            .command([dshot_encoder::throttle_minimum(false); 4]);
    }

    async fn set_reverse_dir(&mut self, rev: [bool; 4]) {
        self.inner
            .command(rev.map(|rev| dshot_encoder::reverse(rev)));
    }

    async fn make_beep(&mut self) {
        // Library does not support beeping yet
    }
}

impl MotorDriver {
    pub fn setup(self, _dshot: DshotConfig) -> impl OutputGroup {
        bind_interrupts!( struct Pio0Irqs {
            PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<peripherals::PIO0>;
        });

        PioMotors {
            inner: crate::dshot_pio::DshotPio::<4, _>::new(
                self.pio,
                Pio0Irqs,
                self.pin_1,
                self.pin_2,
                self.pin_3,
                self.pin_4,
                (52, 0), // eh..
            ),
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn motor_governor(motors: MotorDriver, dshot_cfg: DshotConfig) -> ! {
    let motors = motors.setup(dshot_cfg);
    common::tasks::motor_governor::main(motors).await
}

// ----------------------------------------------------------
// --------------------- UARTs setup ------------------------
// ----------------------------------------------------------

macro_rules! impl_ring_buffered_usart_setup {
    ($fn_name:ident, $UsartX:ident, $USARTX_IRQ:ident, $USARTX:ident, $tx_size:literal, $rx_size:literal, $buf_rx:path, $buf_tx:path) => {
        #[allow(unused)]
        impl $UsartX {
            pub fn setup<'d>(self, uart_cfg: UartConfig) -> embassy_rp::uart::BufferedUart {
                bind_interrupts!(struct UsartIrq {
                    $USARTX_IRQ => embassy_rp::uart::BufferedInterruptHandler<peripherals::$USARTX>;
                });

                // Provide a static buffer for the ring buffer.
                use static_cell::StaticCell;
                static TX_BUF: StaticCell<[u8; $tx_size]> = StaticCell::new();
                static RX_BUF: StaticCell<[u8; $rx_size]> = StaticCell::new();

                let mut config = embassy_rp::uart::Config::default();
                config.baudrate = uart_cfg.baud;

                let usart = embassy_rp::uart::BufferedUart::new(
                    self.periph,
                    self.tx_pin,
                    self.rx_pin,
                    UsartIrq,
                    TX_BUF.init([0; $tx_size]),
                    RX_BUF.init([0; $rx_size]),
                    config,
                );

                usart
            }
        }

        #[embassy_executor::task]
        pub(crate) async fn $fn_name(usart: $UsartX, uart_cfg: UartConfig, serial_id: &'static str) {
            let usart = usart.setup(uart_cfg);

            let (tx, rx) = usart.split();

            let (mut dev_prod, app_cons) = $buf_tx.claim_reader();
            let (mut dev_cons, app_prod) = $buf_rx.claim_writer();

            let io_stream_raw = IoStreamRaw::new(serial_id, app_cons, app_prod);

            static IO_STREAM_RAW: StaticCell<IoStreamRaw<'static>> = StaticCell::new();
            let io_stream_ref = IO_STREAM_RAW.init(io_stream_raw);

            common::serial::insert(io_stream_ref).unwrap();

            let map_err = |error: embassy_rp::uart::Error| {
                <embassy_rp::uart::Error as embedded_io::Error>::kind(&error).into()
            };

            common::embassy_futures::join::join(
                dev_prod.embedded_io_connect(rx, map_err),
                dev_cons.embedded_io_connect(tx, map_err),
            ).await;

            defmt::warn!("[{}] Stream disconnected unexpectedly", serial_id)
        }
    };
}

use common::grantable_io::GrantableIo;
use static_cell::StaticCell;

static BUF_TX0: GrantableIo<256, EmbeddedIoError> = GrantableIo::new();
static BUF_RX0: GrantableIo<512, EmbeddedIoError> = GrantableIo::new();

static BUF_TX1: GrantableIo<256, EmbeddedIoError> = GrantableIo::new();
static BUF_RX1: GrantableIo<512, EmbeddedIoError> = GrantableIo::new();

impl_ring_buffered_usart_setup!(run_uart0, Uart0, UART0_IRQ, UART0, 32, 128, BUF_RX0, BUF_TX0);
impl_ring_buffered_usart_setup!(run_uart1, Uart1, UART1_IRQ, UART1, 32, 128, BUF_RX1, BUF_TX1);

// ----------------------------------------------------------
// --------------- USB for PC-FW connection -----------------
// ----------------------------------------------------------

#[cfg(feature = "usb")]
pub mod usb {

    use common::embassy_usb::driver::Driver;
    use common::types::device::HardwareInfo;
    use embassy_rp::bind_interrupts;

    use crate::resources::Usb;

    impl Usb {
        pub fn setup(self) -> impl Driver<'static> {
            bind_interrupts!(pub struct UsbIrq {
                USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
            });
            embassy_rp::usb::Driver::new(self.usb, UsbIrq)
        }
    }

    #[embassy_executor::task]
    pub(crate) async fn runner(usb: Usb, info: HardwareInfo) -> ! {
        let usb = usb.setup();
        common::tasks::usb_manager::main(usb, info).await
    }
}
