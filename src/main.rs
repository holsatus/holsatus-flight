#![feature(type_changing_struct_update)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![feature(async_closure)]
#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Executor;
use embassy_time::{Timer, Duration};
use panic_probe as _;
use embassy_futures as _;

// Load module for cross-task channels
mod channels;

// Static i2c mutex for shared-bus functionality
use static_cell::StaticCell;
use embassy_rp::{i2c::I2c,peripherals::I2C1, gpio::Pin, multicore::{Stack, spawn_core1}};

// Import task modules
mod task_blinker;
mod task_attitude_controller;
mod task_motor_governor;
mod task_sbus_reader;
mod task_state_estimator;
mod task_commander;
mod task_motor_mixing;
mod task_flight_detector;
mod task_printer;

// Import misc modules
mod functions;
mod sbus_cmd;
mod imu;
mod cfg;

// Crate stack and executor for CORE1 (high priority attitude loop)
static mut CORE1_STACK: Stack<{1024*16}> = Stack::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create quad-motor PIO runner
    let quad_pio_motors = {
        use dshot_pio::dshot_embassy_rp::DshotPio;
        use embassy_rp::{pio::*,bind_interrupts,peripherals::PIO0};
        bind_interrupts!(struct Pio0Irqs {PIO0_IRQ_0 => InterruptHandler<PIO0>;});
        DshotPio::<4,_>::new(p.PIO0, Pio0Irqs, p.PIN_7, p.PIN_13, p.PIN_12, p.PIN_6, cfg::PIO_DSHOT_SPEED.clk_div())
    };

    // Configure and setup sbus compatible uart RX connection
    let uart_rx_sbus = {
        use embassy_rp::{uart::*,bind_interrupts,peripherals::UART1};
        bind_interrupts!(struct Uart1Irqs {UART1_IRQ => InterruptHandler<UART1>;});
        let mut sbus_uart_config = Config::default();
        sbus_uart_config.baudrate = 100_000;
        sbus_uart_config.data_bits = DataBits::DataBits8;
        sbus_uart_config.stop_bits = StopBits::STOP2;
        sbus_uart_config.parity = Parity::ParityEven;
        sbus_uart_config.invert_rx = true;
        UartRx::new(p.UART1, p.PIN_9,Uart1Irqs, p.DMA_CH0, sbus_uart_config)
    };

    // Configure and setup shared async I2C communication for CORE1
    let async_i2c_core1 = {
        use embassy_rp::{i2c,bind_interrupts};
        bind_interrupts!(struct I2c1Irqs {I2C1_IRQ => i2c::InterruptHandler<I2C1>;});
        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, I2c1Irqs, i2c_config)
    };

    // Spawning of system tasks on CORE1 (high priority)

    Timer::after(Duration::from_millis(50)).await;
    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = CORE1_EXECUTOR.init(Executor::new());
        executor1.run(|spawner| {

            use crate::imu::imu_driver;
            spawner.must_spawn(imu_driver(
                spawner.clone(),
                async_i2c_core1,
                unwrap!(channels::IMU_READING.publisher()),
            ));

            use crate::task_state_estimator::state_estimator;
            spawner.must_spawn(state_estimator(
                unwrap!(channels::IMU_READING.subscriber()), 
                unwrap!(channels::ATTITUDE_SENSE.publisher()), 
            ));

            use crate::task_attitude_controller::attitude_controller;
            spawner.must_spawn(attitude_controller(
                unwrap!(channels::ATTITUDE_SENSE.subscriber()),
                unwrap!(channels::ATTITUDE_INT_RESET.subscriber()),
                unwrap!(channels::ATTITUDE_STAB_MODE.subscriber()),
                unwrap!(channels::ATTITUDE_ACTUATE.publisher()),
            ));

            use crate::task_motor_mixing::motor_mixing;
            spawner.must_spawn(motor_mixing(
                unwrap!(channels::THRUST_ACTUATE.subscriber()),
                unwrap!(channels::ATTITUDE_ACTUATE.subscriber()),
                unwrap!(channels::MOTOR_ARM.subscriber()),
                unwrap!(channels::MOTOR_SPIN_CHECK.subscriber()),
                unwrap!(channels::MOTOR_SPEED.publisher()),
            ));

            use crate::task_motor_governor::motor_governor;
            spawner.must_spawn(motor_governor(
                unwrap!(channels::MOTOR_SPEED.subscriber()),
                unwrap!(channels::MOTOR_DIR.subscriber()),
                unwrap!(channels::MOTOR_STATE.publisher()),
                quad_pio_motors,
            )); 
        });
    });

    // Spawning of system tasks on CORE0 (low priority)

    use crate::task_commander::commander;
    spawner.must_spawn(commander(
        unwrap!(channels::SBUS_CMD.subscriber()), 
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::ATTITUDE_SENSE.subscriber()), 
        unwrap!(channels::FLIGHT_MODE.subscriber()),
        unwrap!(channels::MOTOR_ARM.publisher()), 
        unwrap!(channels::MOTOR_DIR.publisher()),
        unwrap!(channels::THRUST_ACTUATE.publisher()),
        unwrap!(channels::BLINKER_MODE.publisher()),
        unwrap!(channels::ATTITUDE_INT_RESET.publisher()),
        unwrap!(channels::ATTITUDE_STAB_MODE.publisher()),
        unwrap!(channels::MOTOR_SPIN_CHECK.publisher()),
    ));

    use crate::task_sbus_reader::sbus_reader;
    spawner.must_spawn(sbus_reader(
        uart_rx_sbus,
        unwrap!(channels::SBUS_CMD.publisher()),
    ));

    use crate::task_blinker::blinker;
    spawner.must_spawn(blinker(
        unwrap!(channels::BLINKER_MODE.subscriber()),
        p.PIN_25.degrade()
    ));

    use crate::task_flight_detector::flight_detector;
    spawner.must_spawn(flight_detector(
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::FLIGHT_MODE.publisher())
    ));

    use crate::task_printer::printer;
    spawner.must_spawn(printer(
        unwrap!(channels::IMU_READING.subscriber()),
        unwrap!(channels::ATTITUDE_SENSE.subscriber()),
        unwrap!(channels::ATTITUDE_ACTUATE.subscriber()),
        unwrap!(channels::ATTITUDE_STAB_MODE.subscriber()),
        unwrap!(channels::THRUST_ACTUATE.subscriber()),
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::FLIGHT_MODE.subscriber()),
    ));

    channels::assert_all_subscribers_used()

}
