#![allow(incomplete_features)]
#![feature(type_changing_struct_update)]
#![feature(type_alias_impl_trait)]
#![feature(generic_const_exprs)]
#![feature(async_fn_in_trait)]
#![feature(async_closure)]
#![no_std]
#![no_main]

use config::definitions::Configuration;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures as _;
use embassy_time::{Timer, Duration};
use panic_probe as _;

// Load module for cross-task channels
mod channels;

// Static i2c mutex for shared-bus functionality
use embassy_rp::{
    gpio::Pin,
    i2c::I2c,
};
use static_cell::StaticCell;

// Import task modules
mod task_attitude_controller;
mod task_blinker;
mod task_commander;
mod task_flight_detector;
mod task_motor_governor;
mod task_motor_mixing;
mod task_printer;
mod task_sbus_reader;
mod task_state_estimator;

// Import misc modules
mod functions;
mod imu;
mod mag;
mod sbus_cmd;
mod drivers;
mod config;
mod mavlink;

// Crate stack and executor for CORE1 (high priority attitude loop)
#[cfg(feature = "dual-core")]
mod core1 {
    use static_cell::StaticCell;
    use embassy_executor::Executor;
    use embassy_rp::multicore::Stack;
    pub static mut STACK: Stack<{ 1024 * 16 }> = Stack::new();
    pub static EXECUTOR: StaticCell<Executor> = StaticCell::new();
}

static CONFIG: StaticCell<Configuration> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    Timer::after(Duration::from_millis(1000)).await;

    // Setup flash storage object
    let mut flash = {
        use embassy_rp::flash::{Flash,Async};
        Flash::<_, Async, { config::definitions::FLASH_AMT }>::new(p.FLASH, p.DMA_CH2)
    };

    // Grab static immutable reference to loaded config
    let config_ref = &*CONFIG.init(config::load_config(&mut flash));
    println!("{}",Debug2Format(&config_ref));

    // Spawn task to handle flash storage of configuration
    spawner.must_spawn(config::config_master(
        flash,
        unwrap!(channels::IMU0_FEATURES.subscriber()),
        unwrap!(channels::MAG0_FEATURES.subscriber()),
    ));

    // Create quad-motor PIO runner
    let quad_pio_motors = {
        use dshot_pio::dshot_embassy_rp::DshotPio;
        use embassy_rp::{bind_interrupts, peripherals::PIO0, pio::*};
        bind_interrupts!(struct Pio0Irqs {PIO0_IRQ_0 => InterruptHandler<PIO0>;});
        DshotPio::<4, _>::new(
            p.PIO0,
            Pio0Irqs,
            p.PIN_10,
            p.PIN_11,
            p.PIN_12,
            p.PIN_13,
            config_ref.dshot_speed.clk_div(),
        )
    };

    // Configure and setup sbus compatible uart RX connection
    let uart_rx_sbus = {
        use embassy_rp::{bind_interrupts, peripherals::UART1, uart::*};
        bind_interrupts!(struct Uart1Irqs {UART1_IRQ => InterruptHandler<UART1>;});
        let mut sbus_uart_config = Config::default();
        sbus_uart_config.baudrate = 100_000;
        sbus_uart_config.data_bits = DataBits::DataBits8;
        sbus_uart_config.stop_bits = StopBits::STOP2;
        sbus_uart_config.parity = Parity::ParityEven;
        sbus_uart_config.invert_rx = true;
        UartRx::new(p.UART1, p.PIN_9, Uart1Irqs, p.DMA_CH0, sbus_uart_config)
    };

    let uart_mavlink = {
        use embassy_rp::{uart::*,bind_interrupts,peripherals::UART0};
        bind_interrupts!(struct Uart0Irqs {UART0_IRQ => InterruptHandler<UART0>;});
        Uart::new(p.UART0, p.PIN_0, p.PIN_1,Uart0Irqs, p.DMA_CH3, p.DMA_CH4, Config::default())
    };

    // Configure and setup shared async I2C communication for CORE1
    let async_i2c_imu = {
        use embassy_rp::{bind_interrupts, i2c,peripherals::I2C0};
        bind_interrupts!(struct I2c0Irqs {I2C0_IRQ => i2c::InterruptHandler<I2C0>;});
        let mut i2c_config = i2c::Config::default();
        i2c_config.frequency = 400_000; // High speed i2c
        I2c::new_async(p.I2C0, p.PIN_17, p.PIN_16, I2c0Irqs, i2c_config)
    };

    // Configure ADC for reading battery state (not implemented)
    let (
        _adc,
        _battery_read_ch,
        _current_read_ch
    ) = {
        use embassy_rp::{bind_interrupts,adc::{InterruptHandler,self},gpio::Pull};
        bind_interrupts!(struct AdcIrqs {ADC_IRQ_FIFO => InterruptHandler;});
        (
            adc::Adc::new(p.ADC, AdcIrqs, Default::default()),
            adc::Channel::new_pin(p.PIN_26, Pull::None),
            adc::Channel::new_pin(p.PIN_27, Pull::None)
        )
    };

    // Define high priority tasks
    let high_prio_tasks = |spawner:Spawner| {
        spawner.must_spawn(crate::imu::imu_master(
            spawner.clone(),
            async_i2c_imu,
            config_ref,
            unwrap!(channels::DO_GYRO_CAL.subscriber()),
            unwrap!(channels::IMU_READING.publisher()),
            unwrap!(channels::IMU0_FEATURES.publisher()),
        ));

        spawner.must_spawn(crate::mag::mag_master(
            config_ref,
            unwrap!(channels::DO_MAG_CAL.subscriber()),
            unwrap!(channels::MAG_READING.publisher()),
            unwrap!(channels::MAG0_FEATURES.publisher()),
        ));

        spawner.must_spawn(crate::task_state_estimator::state_estimator(
            unwrap!(channels::IMU_READING.subscriber()),
            unwrap!(channels::MAG_READING.subscriber()),
            unwrap!(channels::ATTITUDE_SENSE.publisher()),
        ));

        spawner.must_spawn(crate::task_attitude_controller::attitude_controller(
            unwrap!(channels::ATTITUDE_SENSE.subscriber()),
            unwrap!(channels::ATTITUDE_INT_ENABLE.subscriber()),
            unwrap!(channels::ATTITUDE_STAB_MODE.subscriber()),
            unwrap!(channels::ATTITUDE_ACTUATE.publisher()),
        ));

        spawner.must_spawn(crate::task_motor_mixing::motor_mixing(
            unwrap!(channels::THRUST_ACTUATE.subscriber()),
            unwrap!(channels::ATTITUDE_ACTUATE.subscriber()),
            unwrap!(channels::MOTOR_ARM.subscriber()),
            unwrap!(channels::MOTOR_SPIN_CHECK.subscriber()),
            unwrap!(channels::MOTOR_SPEED.publisher()),
        ));

        spawner.must_spawn(crate::task_motor_governor::motor_governor(
            unwrap!(channels::MOTOR_SPEED.subscriber()),
            unwrap!(channels::MOTOR_DIR.subscriber()),
            unwrap!(channels::MOTOR_STATE.publisher()),
            quad_pio_motors,
        ));
    };

    // Spawning of high priority tasks on CORE1
    #[cfg(feature = "dual-core")] {
        use embassy_rp::multicore::spawn_core1;
        use embassy_executor::Executor;
        spawn_core1(p.CORE1, unsafe { &mut core1::STACK }, move || {
            let executor1 = core1::EXECUTOR.init(Executor::new());
            executor1.run(|spawner_core1| { high_prio_tasks(spawner_core1) });
        });
    }

    // Spawning of high priority tasks on CORE0
    #[cfg(not(feature = "dual-core"))]
    high_prio_tasks(spawner);

    // Spawning of low priority tasks on CORE0
    spawner.must_spawn(crate::task_commander::commander(
        spawner.clone(),
        config_ref,
        unwrap!(channels::SBUS_CMD.subscriber()),
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::ATTITUDE_SENSE.subscriber()),
        unwrap!(channels::FLIGHT_MODE.subscriber()),
        unwrap!(channels::MOTOR_ARM.publisher()),
        unwrap!(channels::MOTOR_DIR.publisher()),
        unwrap!(channels::THRUST_ACTUATE.publisher()),
        unwrap!(channels::BLINKER_MODE.publisher()),
        unwrap!(channels::ATTITUDE_INT_ENABLE.publisher()),
        unwrap!(channels::ATTITUDE_STAB_MODE.publisher()),
        unwrap!(channels::MOTOR_SPIN_CHECK.publisher()),
        unwrap!(channels::DO_GYRO_CAL.publisher()),
        unwrap!(channels::DO_MAG_CAL.publisher())
    ));

    spawner.must_spawn(mavlink::mavlink_task(
        spawner.clone(),
        config_ref,
        uart_mavlink,
    ));

    spawner.must_spawn(crate::task_sbus_reader::sbus_reader(
        uart_rx_sbus,
        unwrap!(channels::SBUS_CMD.publisher()),
    ));

    spawner.must_spawn(crate::task_blinker::blinker(
        unwrap!(channels::BLINKER_MODE.subscriber()),
        p.PIN_25.degrade(),
    ));

    spawner.must_spawn(crate::task_flight_detector::flight_detector(
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::FLIGHT_MODE.publisher()),
    ));

    spawner.must_spawn(crate::task_printer::printer(
        unwrap!(channels::IMU_READING.subscriber()),
        unwrap!(channels::MAG_READING.subscriber()),
        unwrap!(channels::ATTITUDE_SENSE.subscriber()),
        unwrap!(channels::ATTITUDE_ACTUATE.subscriber()),
        unwrap!(channels::ATTITUDE_STAB_MODE.subscriber()),
        unwrap!(channels::THRUST_ACTUATE.subscriber()),
        unwrap!(channels::MOTOR_STATE.subscriber()),
        unwrap!(channels::FLIGHT_MODE.subscriber()),
    ));

    channels::assert_all_subscribers_used()
}
