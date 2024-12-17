use assign_resources::assign_resources;
use common::{
    drivers::imu::ImuConfig, embassy_usb::driver::Driver, hw_abstraction::FourMotors, types::{config::{DshotConfig, I2cConfig, UartConfig}, device::HardwareInfo}
};
use dshot_pio::DshotPioTrait;
use embassy_rp::{
    bind_interrupts, peripherals,
};
use embedded_hal_async::i2c::I2c;

use crate::config::gen_default_cfg;

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

assign_resources! {
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

// ----------------------------------------------------------
// -------------------- Main IMU I2C ------------------------
// ----------------------------------------------------------

impl I2c0 {
    pub fn setup(self, cfg: &I2cConfig) -> impl I2c {
        bind_interrupts!(struct I2c0Irq {
            I2C0_IRQ => embassy_rp::i2c::InterruptHandler<peripherals::I2C0>;
        });

        let mut config = embassy_rp::i2c::Config::default();
        config.frequency = cfg.frequency;
        embassy_rp::i2c::I2c::new_async(
            self.periph,
            self.scl,
            self.sda,
            I2c0Irq,
            config,
        )
    }
}

#[embassy_executor::task]
pub(crate) async fn imu_reader_6dof(i2c: I2c0, i2c_cfg: &'static I2cConfig, imu_cfg: &'static ImuConfig) -> ! {
    let i2c = i2c.setup(i2c_cfg);
    common::tasks::imu_reader::main_6dof_i2c(i2c, imu_cfg, Some(0x69)).await
}

// ----------------------------------------------------------
// ---------------- FLASH config storage --------------------
// ----------------------------------------------------------

impl Flash {
    pub fn setup<'d>(self) -> impl common::embedded_storage_async::nor_flash::NorFlash {
        embassy_rp::flash::Flash::<_, _, { 2 * 1024 * 1024 }>::new(self.periph, self.dma)
    }
}


#[embassy_executor::task]
pub(crate) async fn configurator(flash: Flash) -> ! {
    let flash = flash.setup();
    common::tasks::configurator::main(flash, 0..{2 * 1024 * 1024}, gen_default_cfg).await
}

// ----------------------------------------------------------
// --------------- PWM-based motor driver -------------------
// ----------------------------------------------------------

struct PioMotors<'a, PIO: embassy_rp::pio::Instance> {
    inner: dshot_pio::dshot_embassy_rp::DshotPio<'a, 4, PIO>
}

impl <'a, PIO: embassy_rp::pio::Instance> FourMotors for PioMotors<'a, PIO> {
    async fn set_motor_speeds(&mut self, speeds: [u16; 4]) {
        self.inner.throttle_clamp(speeds);
    }

    async fn set_motor_speeds_min(&mut self) {
        self.inner.throttle_minimum();
    }

    async fn set_reverse_dir(&mut self, rev: [bool; 4]) {
        self.inner.reverse(rev);
    }

    async fn make_beep(&mut self) {
        // Library does not support beeping yet
    }
}

impl MotorDriver {
    pub fn setup(self, _dshot: &DshotConfig) -> impl FourMotors {
        bind_interrupts!( struct Pio0Irqs {
            PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<peripherals::PIO0>;
        });

        PioMotors{ inner: dshot_pio::dshot_embassy_rp::DshotPio::<4, _>::new(
            self.pio,
            Pio0Irqs,
            self.pin_1,
            self.pin_2,
            self.pin_3,
            self.pin_4,
            (52, 0)
        )}
    }
}

#[embassy_executor::task]
pub(crate) async fn motor_governor(motors: MotorDriver, dshot_cfg: &'static DshotConfig) -> ! {
    let motors = motors.setup(dshot_cfg);
    common::tasks::motor_governor::main(motors).await
}

// ----------------------------------------------------------
// --------------------- UARTs setup ------------------------
// ----------------------------------------------------------

macro_rules! impl_ring_buffered_usart_setup {
    ($UsartX:ident, $USARTX_IRQ:ident, $USARTX:ident, $tx_size:literal, $rx_size:literal) => {
        #[allow(unused)]
        impl $UsartX {
            pub fn setup<'d>(self, uart_cfg: &UartConfig) -> embassy_rp::uart::BufferedUart<'d, peripherals::$USARTX> {
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
                    UsartIrq,
                    self.tx_pin,
                    self.rx_pin,
                    TX_BUF.init([0; $tx_size]),
                    RX_BUF.init([0; $rx_size]),
                    config,
                );

                usart
            }
        }
    };
}

impl_ring_buffered_usart_setup!(Uart0, UART0_IRQ, UART0, 32, 128);
impl_ring_buffered_usart_setup!(Uart1, UART1_IRQ, UART1, 32, 128);

// RC controls (e.g. sbus/crsf)
#[embassy_executor::task]
pub(crate) async fn rc_serial_read(usart: Uart0, uart_cfg: &'static UartConfig) -> ! {
    let usart = usart.setup(uart_cfg);
    let (_tx, mut rx) = usart.split();
    common::tasks::rc_reader::main(&mut rx).await
}

// GNSS module (ubx protocol)
#[embassy_executor::task]
pub(crate) async fn gnss_serial_read(usart: Uart1, uart_cfg: &'static UartConfig) -> ! {
    let usart = usart.setup(uart_cfg);
    let (_tx, mut rx) = usart.split();
    common::tasks::gnss_reader::main(&mut rx).await
}

// ----------------------------------------------------------
// --------------- USB for PC-FW connection -----------------
// ----------------------------------------------------------

impl Usb {
    pub fn setup(self) -> impl Driver<'static> {
        bind_interrupts!(pub struct UsbIrq {
            USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
        });
        embassy_rp::usb::Driver::new(self.usb, UsbIrq)
    }
}

#[embassy_executor::task]
pub(crate) async fn usb_handler(usb: Usb, info: &'static HardwareInfo) -> ! {
    let usb = usb.setup();
    common::tasks::usb_manager::main(usb, info).await
}
