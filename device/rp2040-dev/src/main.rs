#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

mod resources;

mod config;
mod dshot_pio;

defmt::timestamp!(
    "{=u64:us}",
    common::embassy_time::Instant::now().as_micros()
);

/// Helper macro to create interrupt executors.
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_rp::interrupt;
        use embassy_rp::interrupt::{InterruptExt, Priority};

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
    let p = embassy_rp::init(resources::config());
    let r = resources::split(p);
    common::embassy_time::Timer::after_millis(10).await;

    defmt::info!("[rp2040-dev]: clocks initialized, starting tasks");

    // The native sampling time of the ICM20948
    common::signals::CONTROL_FREQUENCY.store(1125, Ordering::Relaxed);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(SWI_IRQ_0, P1);
    let level_1_spawner = interrupt_executor!(SWI_IRQ_1, P2);

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(resources::param_storage(r.flash).unwrap());

    // Give special priority to the serial port used as primary RC input
    level_0_spawner.spawn(resources::run_uart0(r.uart_0, config::uart0(), "uart0").unwrap());
    level_1_spawner.spawn(resources::run_uart1(r.uart_1, config::uart1(), "uart1").unwrap());

    // The peripheral must run at a higher priority than the shell task
    #[cfg(feature = "usb")]
    {
        // Note: We run the peripheral at a higher priority because the shell
        // parser operates on a blocking Write implementation. This way the
        // code will nenver truly block, since the write results in an interrupt.
        level_1_spawner.spawn(resources::usb::runner(r.usb, config::hw_info()).unwrap());
        level_t_spawner.spawn(common::shell::main("usb").unwrap());
    }

    // ------------------ high-priority tasks -------------------

    level_0_spawner.spawn(resources::imu_reader(r.i2c_0, config::i2c1(), config::imu()).unwrap());
    level_0_spawner.spawn(resources::motor_governor(r.motors, config::dshot()).unwrap());

    level_0_spawner.spawn(common::tasks::rc_reader::main("uart0").unwrap());
    level_0_spawner.spawn(common::tasks::rc_binder::main().unwrap());
    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    #[cfg(feature = "gnss")]
    level_1_spawner.spawn(common::tasks::gnss_reader::main("uart1").unwrap());
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());
    level_t_spawner.spawn(common::tasks::in_flight_estimator::main().unwrap());

    // -------------------------- fin ---------------------------
    common::embassy_time::Timer::after_secs(1).await;

    loop {
        defmt::debug!("[rp2040-dev] thread-mode execution operating as intended");
        common::embassy_time::Timer::after_secs(10).await;
    }
}
