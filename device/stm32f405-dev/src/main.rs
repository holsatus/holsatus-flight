#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod resource_setup;
use resource_setup::*;

mod config;
mod dshot_pwm;

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
    const ID: &str = "stm32f405_main";

    // ---------------------- early setup -----------------------

    // Initialize the chip and split the resources
    let p = config::initialize();
    let r = split_resources!(p);
    common::embassy_time::Timer::after_millis(10).await;

    defmt::info!("{}: clocks initialized, starting tasks", ID);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(CAN1_RX0, P10);
    let level_1_spawner = interrupt_executor!(CAN1_RX1, P11);

    // We must spawn the configurator first to obtain the boot-config
    level_t_spawner.spawn(configurator(r.flash).unwrap());
    let config = common::signals::BOOT_CONFIG.get().await;

    // ------------------ high-priority tasks -------------------

    if let (Some(i2c_cfg), Some(imu_cfg)) = (&config.i2c1, &config.imu0) {
        level_0_spawner.spawn(imu_reader(r.i2c_1, r.int_pin, i2c_cfg, imu_cfg).unwrap());
    } else {
        defmt::error!("{}: No I2C1 and/or IMU0 config found", ID);
    }

    if let Some(uart_cfg) = &config.uart1 {
        level_0_spawner.spawn(rc_serial_read(r.usart_1.any(), uart_cfg).unwrap());
    } else {
        defmt::error!("{}: No USART1 config found", ID);
    }

    if let Some(motor_cfg) = &config.motors {
        level_0_spawner.spawn(motor_governor(r.motors, motor_cfg).unwrap());
    } else {
        defmt::error!("{}: No motor protocol config found", ID)
    }

    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::rate_loop::main().unwrap());
    level_0_spawner.spawn(common::tasks::rc_mapper::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    if let Some(uart_cfg) = &config.uart2 {
        level_1_spawner.spawn(gnss_serial_read(r.usart_6.any(), uart_cfg).unwrap());
    } else {
        defmt::error!("{}: No USART2 config found", ID);
    }

    level_1_spawner.spawn(common::tasks::commander::commander_entry_task().unwrap());
    level_1_spawner.spawn(common::tasks::att_estimator::main().unwrap());
    level_1_spawner.spawn(common::tasks::angle_loop::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    if let Some(sdmmc_cfg) = &config.sdmmc {
        level_t_spawner.spawn(blackbox_fat(r.sdcard, sdmmc_cfg).unwrap());
    } else {
        defmt::error!("{}: No SDMMC config found", ID);
    }

    if let Some(uart_cfg) = &config.uart3 {
        level_t_spawner.spawn(mavlink_serial(r.usart_3.any(), uart_cfg).unwrap())
    } else {
        defmt::error!("{}: No USART3 config found", ID);
    }

    // A valid config always exists for the usb-handler otherwise
    // the user will not be able to interface with the system.
    level_t_spawner.spawn(usb_manager(r.usb, &config.info).unwrap());
    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());

    // -------------------------- fin ---------------------------
}
