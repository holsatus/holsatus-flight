#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

mod config;
mod dshot_pwm;
mod resources;

/// Helper macro to create an interrupt executor.
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::{InterruptExt, Priority};

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            EXECUTOR.on_interrupt()
        }

        spawner
    }};
}

#[embassy_executor::main]
async fn main(level_t_spawner: embassy_executor::Spawner) {
    // ---------------------- early setup -----------------------

    // Initialize the chip and split the resources
    let p = config::initialize();
    let r = resources::assign(p);
    common::embassy_time::Timer::after_millis(10).await;

    defmt::info!("[stm32f405-dev] clocks initialized, starting tasks");

    common::signals::CONTROL_FREQUENCY.store(500, Ordering::Relaxed);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(CAN1_RX0, P10);
    let level_1_spawner = interrupt_executor!(CAN1_RX1, P11);

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(resources::param_storage(r.flash, config::flash()).unwrap());

    // Give special priority to the serial port used as primary input
    level_0_spawner.spawn(resources::run_usart1(r.usart_1, config::usart1(), "serial0").unwrap()); // CRSF
    level_1_spawner.spawn(resources::run_usart3(r.usart_3, config::usart3(), "serial1").unwrap()); // MAVLINK
    level_1_spawner.spawn(resources::run_usart6(r.usart_6, config::usart6(), "serial2").unwrap()); // GNSS

    #[cfg(feature = "usb")]
    {
        // Note: We run the peripheral at a higher priority because the shell
        // parser operates on a blocking Write implementation. This way the
        // code will enver truly block, since the shell results in an interrupt.
        level_1_spawner.spawn(resources::usb::runner(r.usb, config::hwinfo()).unwrap());
        level_t_spawner.spawn(common::shell::main("usb").unwrap());
    }

    #[cfg(feature = "sdmmc")]
    level_t_spawner.spawn(resources::sdmmc::blackbox_fat(r.sdcard, config::sdmmc()).unwrap());

    common::embassy_time::Timer::after_millis(1).await;

    // ------------------ high-priority tasks -------------------

    // These take direct ownership of their hardware to avoid additional complexity
    level_0_spawner.spawn(resources::imu_reader(r.i2c_1, config::i2c1(), config::imu()).unwrap());
    level_0_spawner.spawn(resources::motor_governor(r.motors, config::motor()).unwrap());

    level_0_spawner.spawn(common::tasks::rc_reader::main("serial0").unwrap());
    level_0_spawner.spawn(common::tasks::rc_binder::main().unwrap());
    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    level_1_spawner.spawn(common::tasks::gnss_reader::main("serial2").unwrap());
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::att_estimator::main().unwrap());
    level_1_spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    #[cfg(feature = "mavlink")]
    level_t_spawner.spawn(common::mavlink2::main("serial1").unwrap());

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());
    level_t_spawner.spawn(common::tasks::controller_mpc::main().unwrap());

    // Bogus schmogus
    common::signals::VICON_POSITION_ESTIMATE.send(common::types::measurements::ViconData {
        timestamp_us: 100,
        position: [0.0; 3],
        pos_var: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        attitude: [0.0; 3],
        att_var: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    });

    // -------------------------- fin ---------------------------
    common::embassy_time::Timer::after_secs(1).await;

    loop {
        defmt::debug!("[stm32f405-dev] thread-mode execution operating as intended");
        common::embassy_time::Timer::after_secs(10).await;
    }
}
