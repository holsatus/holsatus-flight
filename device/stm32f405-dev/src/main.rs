#![no_std]
#![no_main]

use common::embassy_time::Timer;
use defmt_rtt as _;
use panic_probe as _;

mod resource_setup;
use resource_setup::*;

mod config;
mod dshot_pwm;

defmt::timestamp!("{=u64:us}", common::embassy_time::Instant::now().as_micros());

/// Helper macro to create interrupt executors.
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
    const ID: &str = "stm32f405_main";

    // ---------------------- early setup -----------------------

    // Initialize the chip and split the resources
    let p = config::initialize();
    let r = split_resources!(p);
    Timer::after_millis(10).await;

    defmt::info!("{}: clocks initialized, starting tasks", ID);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(UART4, P5);
    let level_1_spawner = interrupt_executor!(UART5, P6);

    // We must spawn the configurator first to obtain the boot-config
    level_t_spawner.must_spawn(configurator(r.flash));
    let config = common::signals::BOOT_CONFIG.get().await;

    // ------------------ high-priority tasks -------------------

    if let (Some(i2c_cfg), Some(imu_cfg)) = (&config.i2c1, &config.imu0) {
        level_0_spawner.must_spawn(imu_reader_6dof(r.i2c_1, r.int_pin, i2c_cfg, imu_cfg));
    } else {
        defmt::error!("{}: No I2C1 and/or IMU0 config found", ID);
    }

    if let Some(uart_cfg) = &config.uart1 {
        level_0_spawner.must_spawn(rc_serial_read(r.usart_1.any(), uart_cfg));
    } else {
        defmt::error!("{}: No USART1 config found", ID);
    }

    if let Some(motor_cfg) = &config.motors {
        level_0_spawner.must_spawn(motor_governor(r.motors, motor_cfg));
    } else {
        defmt::error!("{}: No motor protocol config found", ID)
    }

    level_0_spawner.must_spawn(common::tasks::signal_router::main());
    level_0_spawner.must_spawn(common::tasks::rate_loop::main());
    level_0_spawner.must_spawn(common::tasks::rc_mapper::main());

    // ----------------- medium-priority tasks ------------------

    if let Some(uart_cfg) = &config.uart2 {
        level_1_spawner.must_spawn(gnss_serial_read(r.usart_6.any(), uart_cfg));
    } else {
        defmt::error!("{}: No USART2 config found", ID);
    }

    level_1_spawner.must_spawn(common::tasks::commander::main());
    level_1_spawner.must_spawn(common::tasks::att_estimator::main());
    level_1_spawner.must_spawn(common::tasks::angle_loop::main());

    // ------------------- Low-priority tasks -------------------

    if let Some(sdmmc_cfg) = &config.sdmmc {
        level_t_spawner.must_spawn(blackbox_fat(r.sdcard, sdmmc_cfg));
    } else {
        defmt::error!("{}: No SDMMC config found", ID);
    }

    if let Some(uart_cfg) = &config.uart3 {
        level_t_spawner.must_spawn(mavlink_serial(r.usart_3.any(), uart_cfg))
    } else {
        defmt::error!("{}: No USART3 config found", ID);
    }

    // A valid config always exists for the usb-handler otherwise
    // the user will not be able to interface with the system.
    level_t_spawner.must_spawn(usb_manager(r.usb, &config.info));
    level_t_spawner.must_spawn(common::tasks::calibrator::main());
    level_t_spawner.must_spawn(common::tasks::arm_blocker::main());
    // level_t_spawner.must_spawn(common::tasks::signal_stats::main());

    // -------------------------- fin ---------------------------
}
