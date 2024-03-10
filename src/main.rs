#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_futures as _;
use panic_probe as _;

use embassy_rp::config::Config;
use static_cell::StaticCell;

// Static reference to the SPI0 bus
use embassy_rp::{
    peripherals::SPI0,
    spi::{Async, Spi},
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, Spi<SPI0, Async>>> = StaticCell::new();

// Suggested fix to prevent deadlock after flashing on the RP2040
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    // Reset spinlock 31
    core::arch::asm!(
        "
        ldr r0, =1
        ldr r1, =0xd000017c
        str r0, [r1]
    "
    );
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

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize the RP2040 with default configuration
    #[cfg(not(feature = "overclock"))]
    let p = {
        info!("Initializing RP2040 with default configuration");
        embassy_rp::init(Config::default())
    };

    // Initialize the RP2040 with 2x overclock
    #[cfg(feature = "overclock")]
    let p = {
        info!("Initializing RP2040 with 2x overclock");
        embassy_rp::init(Config::overclock())
    };

    info!("[MAIN]: Starting main task, 2 second delay...");
    use embassy_time::{Duration, Timer};
    Timer::after(Duration::from_secs(2)).await;

    // Setup flash storage object
    let flash = {
        use embassy_rp::flash::{Async, Flash};
        Flash::<_, Async, { holsatus_flight::config::FLASH_AMT }>::new(p.FLASH, p.DMA_CH0)
    };

    // Load the configuration from flash and initialize the static reference
    spawner.must_spawn(holsatus_flight::t_config_master::config_master(flash));

    // Create quad-motor PIO runner
    let driver_dshot_pio = {
        use holsatus_flight::drivers::rp2040::dshot_pio::DshotPio;
        use embassy_rp::{bind_interrupts, peripherals::PIO0, pio::*};
        bind_interrupts!(struct Pio0Irqs {PIO0_IRQ_0 => InterruptHandler<PIO0>;});
        let speed = holsatus_flight::messaging::CFG_DSHOT_SPEED.spin_get().await;
        DshotPio::<4, _>::new(p.PIO0, Pio0Irqs, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13, speed.clk_div(),
        )
    };

    // Configure and setup sbus compatible uart RX connection
    let uart_rx_sbus = {
        use embassy_rp::uart::{Config, DataBits, Parity, StopBits};
        use embassy_rp::{bind_interrupts, peripherals::UART1, uart::*};
        bind_interrupts!(struct Uart1Irqs {UART1_IRQ => InterruptHandler<UART1>;});
        let mut sbus_uart_config = Config::default();
        sbus_uart_config.baudrate = 100_000;
        sbus_uart_config.data_bits = DataBits::DataBits8;
        sbus_uart_config.stop_bits = StopBits::STOP2;
        sbus_uart_config.parity = Parity::ParityEven;
        sbus_uart_config.invert_rx = true;
        UartRx::<_, Async>::new(p.UART1, p.PIN_9, Uart1Irqs, p.DMA_CH1, sbus_uart_config)
    };

    // Configure and setup Mavlink-compatible UART connection
    let _uart_mavlink = {
        use embassy_rp::{bind_interrupts, peripherals::UART0, uart::*};
        bind_interrupts!(struct Uart0Irqs {UART0_IRQ => InterruptHandler<UART0>;});
        let mut uart_config = Config::default();
        uart_config.baudrate = 1_000_000;
        Uart::new(p.UART0, p.PIN_0, p.PIN_1, Uart0Irqs, p.DMA_CH2, p.DMA_CH3, uart_config)
    };

    // Configure and setup shared async I2C communication for CORE0
    let i2c_async = {
        use embassy_rp::{bind_interrupts, i2c, peripherals::I2C0};
        bind_interrupts!(struct I2c0Irqs {I2C0_IRQ => i2c::InterruptHandler<I2C0>;});
        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        i2c::I2c::new_async(p.I2C0, p.PIN_17, p.PIN_16, I2c0Irqs, i2c_config)
    };

    // Configure and setup shared async SPI communication for CORE0
    let _async_spi_imu = {
        let mut spi_config = embassy_rp::spi::Config::default();
        spi_config.frequency = 7_000_000; // High speed spi
        let spi_bus = embassy_rp::spi::Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH4, p.DMA_CH5, spi_config,
        );
        let static_spi_bus_ref = SPI_BUS.init(Mutex::new(spi_bus));
        let cs_pin = embassy_rp::gpio::Output::new(p.PIN_15, embassy_rp::gpio::Level::High);
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(static_spi_bus_ref, cs_pin)
    };

    // Initilize the core0 main task (sensors, control loop, remote control, etc.)
    spawner.must_spawn(holsatus_flight::main_core0::main_core0(
        i2c_async,
        uart_rx_sbus,
        driver_dshot_pio,
    ));

    use holsatus_flight::common::types::VehicleState;
    holsatus_flight::messaging::VEHICLE_STATE.sender().send(VehicleState::Boot);

    // Initilize the core1 main task (telemetry, some estimators, lower-priority tasks, etc.)

    // // Allocate room for the core1 stack and executor
    // use embassy_rp::multicore::Stack;
    // static CORE1_STACK: StaticCell<Stack<{ 1024 * 4 }>> = StaticCell::new();
    // static CORE1_EXECUTOR: StaticCell<embassy_executor::Executor> = StaticCell::new();

    // use embassy_executor::Executor;
    // use embassy_rp::multicore::spawn_core1;
    // let stack = CORE1_STACK.init(Stack::new());
    // spawn_core1(p.CORE1, stack, move || {
    //     let executor1 = CORE1_EXECUTOR.init(Executor::new());
    //     executor1.run(|spawner| {
    //         spawner.must_spawn(holsatus_flight::main_core1::main_core1(
    //             spawner,
    //             embassy_rp::gpio::Pin::degrade(p.PIN_25),
    //             uart_mavlink,
    //             config,
    //         ))
    //     });
    // });
}