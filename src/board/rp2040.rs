use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;


pub const MACHINE_BOARD: &str = "DevBoard1";
pub const MACHINE_PROCESSOR: &str = "RP2040";

pub const FLASH_AMT: usize = 2 * 1024 * 1024;
pub const PAGE_SIZE: usize = embassy_rp::flash::PAGE_SIZE;

// I2c pheripheral for sensors

type I2cPeriph = peripherals::I2C0;
type I2cSdaPin = peripherals::PIN_17;
type I2cSclPin = peripherals::PIN_16;

bind_interrupts!(struct I2c0Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2cPeriph>;
});

struct AsyncI2c {
    pub i2c: I2cPeriph,
    pub sda_pin: I2cSdaPin,
    pub scl_pin: I2cSclPin,
    pub interrupt: I2c0Irqs,
}

pub type AsyncI2cPeripheral = embassy_rp::i2c::I2c<'static, I2cPeriph, embassy_rp::i2c::Async>;

impl AsyncI2c {
    pub fn init(self) -> AsyncI2cPeripheral {
        let mut i2c_config = embassy_rp::i2c::Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        embassy_rp::i2c::I2c::new_async(self.i2c, self.sda_pin, self.scl_pin, self.interrupt, i2c_config)
    }
}

// UART perihperal for sbus

type UartSbusPeriph = peripherals::UART1;
type UartSbusRxPin = peripherals::PIN_9;
type UartSbusDma = peripherals::DMA_CH1;

bind_interrupts!(struct UartSbusIrqs {
    UART1_IRQ => embassy_rp::uart::InterruptHandler<UartSbusPeriph>;
});

struct UartSbus {
    pub uart: UartSbusPeriph,
    pub rx_pin: UartSbusRxPin,
    pub dma: UartSbusDma,
    pub interrupt: UartSbusIrqs,
}

pub type UartRxSbusPeripheral = embassy_rp::uart::UartRx<'static, UartSbusPeriph, embassy_rp::uart::Async>;

impl UartSbus {
    pub fn init(self) -> UartRxSbusPeripheral {
        use embassy_rp::uart::{Config, DataBits, Parity, StopBits};
        let mut sbus_uart_config = Config::default();
        sbus_uart_config.baudrate = 100_000;
        sbus_uart_config.data_bits = DataBits::DataBits8;
        sbus_uart_config.stop_bits = StopBits::STOP2;
        sbus_uart_config.parity = Parity::ParityEven;
        sbus_uart_config.invert_rx = true;
        embassy_rp::uart::UartRx::new(self.uart, self.rx_pin, self.interrupt, self.dma, sbus_uart_config)
    }
}

// Dshot motor driver

type PioDshotPeriph = peripherals::PIO0;
type PioDshotPin0 = peripherals::PIN_10;
type PioDshotPin1 = peripherals::PIN_11;
type PioDshotPin2 = peripherals::PIN_12;
type PioDshotPin3 = peripherals::PIN_13;

bind_interrupts!(struct PioDshotIqrs {
   PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PioDshotPeriph>; 
});

struct MotorDriver {
    pub pio: PioDshotPeriph,
    pub pin_0: PioDshotPin0,
    pub pin_1: PioDshotPin1,
    pub pin_2: PioDshotPin2,
    pub pin_3: PioDshotPin3,
    pub interrupt: PioDshotIqrs,
}

pub type MotorDriverPeripheral = crate::drivers::rp2040::dshot_pio::DshotPio<'static, 4, PioDshotPeriph>;

impl MotorDriver {
    pub fn init(self) -> MotorDriverPeripheral {
        use crate::config::DshotSpeed;
        crate::drivers::rp2040::dshot_pio::DshotPio::<4, _>::new(
            self.pio, self.interrupt, self.pin_0, self.pin_1, self.pin_2, self.pin_3, Some(DshotSpeed::Dshot300.clk_div())
        )
    }
}

// Flash storage for configuration

type FlashPeriph = peripherals::FLASH;
type FlashDma = peripherals::DMA_CH0;

struct CfgFlash {
    pub flash: FlashPeriph,
    pub dma: FlashDma,
}

pub type FlashPeripheral = embassy_rp::flash::Flash<'static, FlashPeriph, embassy_rp::flash::Async, FLASH_AMT>;

impl CfgFlash {
    pub fn init(self) -> FlashPeripheral {
        embassy_rp::flash::Flash::new(self.flash, self.dma)
    }
}

// UART for Mavlink communication
#[cfg(feature = "mavlink")]
pub use sealed_uart_mavlink::*;
#[cfg(feature = "mavlink")]
mod sealed_uart_mavlink {
    use super::*;

    type UartMavPeriph = peripherals::UART0;
    type UartMavTxPin = peripherals::PIN_0;
    type UartMavRxPin = peripherals::PIN_1;
    type UartMavTxDma = peripherals::DMA_CH2;
    type UartMavRxDma = peripherals::DMA_CH3;

    bind_interrupts!(pub struct UartMavIrqs {
        UART0_IRQ => embassy_rp::uart::InterruptHandler<UartMavPeriph>;
    });

    pub struct UartMav {
        pub uart: UartMavPeriph,
        pub tx_pin: UartMavTxPin,
        pub rx_pin: UartMavRxPin,
        pub tx_dma: UartMavTxDma,
        pub rx_dma: UartMavRxDma,
        pub interrupt: UartMavIrqs,
    }

    pub type UartMavPeripheral = embassy_rp::uart::Uart<'static, UartMavPeriph, embassy_rp::uart::Async>;

    impl UartMav {
        pub fn init(self) -> UartMavPeripheral {
            let mut uart_config = embassy_rp::uart::Config::default();
            uart_config.baudrate = 1_000_000;
            embassy_rp::uart::Uart::new(self.uart, self.tx_pin, self.rx_pin, self.interrupt, self.tx_dma, self.rx_dma, uart_config)
        }
    }
}


// USB peripheral for shell, log off-loading
#[cfg(feature = "shell")]
pub use sealed_shell::*;
#[cfg(feature = "shell")]
mod sealed_shell {
    use super::*;

    type UsbPeriph = peripherals::USB;

    bind_interrupts!(pub struct UsbIrqs {
        USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<UsbPeriph>;
    });

    pub struct Usb {
        pub usb: UsbPeriph,
        pub interrupt: UsbIrqs,
    }

    pub type UsbPeripheral = embassy_rp::usb::Driver<'static, UsbPeriph>;

    impl Usb {
        pub fn init(self) -> UsbPeripheral {
            embassy_rp::usb::Driver::new(self.usb, self.interrupt)
        }
    }
}

// Core 1 peripheral for dual-core support

pub type Core1 = peripherals::CORE1;

// Collection of all pheripherals

struct AllPeripherals {
    pub async_i2c: AsyncI2c,
    pub uart_sbus: UartSbus,
    pub motor_driver: MotorDriver,
    pub cfg_flash: CfgFlash,

    #[cfg(feature = "mavlink")]
    pub uart_mavlink: UartMav,

    #[cfg(feature = "shell")]
    pub usb: Usb,

    #[cfg(feature = "dualcore")]
    pub core1: Core1,
}

#[cfg(feature = "overclock")]
trait Overclock<T> {
    fn overclock() -> T;
}

#[cfg(feature = "overclock")]
impl Overclock<embassy_rp::config::Config> for embassy_rp::config::Config {
    fn overclock() -> Self {
        let mut config = Self::default();
        if let Some(xosc) = config.clocks.xosc.as_mut() {
            if let Some(sys_pll) = xosc.sys_pll.as_mut() {
                sys_pll.post_div2 = 1;
            }
        }
        config
    }
}

fn assemble_peripherals() -> AllPeripherals { 

    defmt::info!("Initializing peripherals");

    #[cfg(not(feature = "overclock"))]
    let p = embassy_rp::init(embassy_rp::config::Config::default());

    // Initialize the RP2040 with 2x overclock
    #[cfg(feature = "overclock")]
    let p = embassy_rp::init(embassy_rp::config::Config::overclock());

    defmt::info!("Peripherals initialized");
    
    let async_i2c = AsyncI2c {
        i2c: p.I2C0,
        sda_pin: p.PIN_17,
        scl_pin: p.PIN_16,
        interrupt: I2c0Irqs,
    };

    let uart_sbus = UartSbus {
        uart: p.UART1,
        rx_pin: p.PIN_9,
        dma: p.DMA_CH1,
        interrupt: UartSbusIrqs,
    };

    let motor_driver = MotorDriver {
        pio: p.PIO0,
        pin_0: p.PIN_10,
        pin_1: p.PIN_11,
        pin_2: p.PIN_12,
        pin_3: p.PIN_13,
        interrupt: PioDshotIqrs,
    };

    let cfg_flash = CfgFlash {
        flash: p.FLASH,
        dma: p.DMA_CH0,
    };

    #[cfg(feature = "mavlink")]
    let uart_mavlink = UartMav {
        uart: p.UART0,
        tx_pin: p.PIN_0,
        rx_pin: p.PIN_1,
        tx_dma: p.DMA_CH2,
        rx_dma: p.DMA_CH3,
        interrupt: UartMavIrqs,
    };

    #[cfg(feature = "shell")]
    let usb = Usb {
        usb: p.USB,
        interrupt: UsbIrqs
    };

    #[cfg(feature = "dualcore")]
    let core1 = p.CORE1;

    AllPeripherals {
        async_i2c,
        uart_sbus,
        motor_driver,
        cfg_flash,

        #[cfg(feature = "mavlink")]
        uart_mavlink,

        #[cfg(feature = "shell")]
        usb,

        #[cfg(feature = "dualcore")]
        core1,
    }
}

pub struct InitializedPeripherals {
    pub async_i2c: AsyncI2cPeripheral,

    pub uart_sbus: UartRxSbusPeripheral,

    pub motor_driver: MotorDriverPeripheral,

    pub cfg_flash: FlashPeripheral,

    #[cfg(feature = "mavlink")]
    pub uart_mavlink: UartMavPeripheral,

    #[cfg(feature = "shell")]
    pub usb: UsbPeripheral,

    #[cfg(feature = "dualcore")]
    pub core1: Core1,
}

pub fn init_peripherals() -> InitializedPeripherals {
    let p = assemble_peripherals();
    InitializedPeripherals {
        async_i2c: p.async_i2c.init(),
        uart_sbus: p.uart_sbus.init(),
        motor_driver: p.motor_driver.init(),
        cfg_flash: p.cfg_flash.init(),

        #[cfg(feature = "mavlink")]
        uart_mavlink: p.uart_mavlink.init(),

        #[cfg(feature = "shell")]
        usb: p.usb.init(),

        #[cfg(feature = "dualcore")]
        core1: p.core1,
    }
}
