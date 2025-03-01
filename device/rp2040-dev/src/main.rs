#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod resource_setup;
use resource_setup::*;

mod config;

defmt::timestamp!("{=u64:us}", common::embassy_time::Instant::now().as_micros());

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
    const ID: &str = "rp2040_main";

    // ---------------------- early setup -----------------------

    // Initialize the chip and split the resources
    let p = embassy_rp::init(resource_setup::config());
    let r = split_resources!(p);
    
    defmt::info!("{}: clocks initialized, starting tasks", ID);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(SWI_IRQ_0, P1);
    let level_1_spawner = interrupt_executor!(SWI_IRQ_1, P2);

    // We must spawn the configurator first to obtain the boot-config
    level_t_spawner.must_spawn(configurator(r.flash));
    let config = common::signals::BOOT_CONFIG.get().await;

    // ------------------ high-priority tasks -------------------

    if let (Some(i2c_cfg), Some(imu_cfg)) = (&config.i2c1, &config.imu0) {
        level_0_spawner.must_spawn(imu_reader_6dof(r.i2c_0, i2c_cfg, imu_cfg));
    } else {
        defmt::error!("{}: No I2C1 and/or IMU0 config found", ID);
    }

    if let Some(uart_cfg) = &config.uart1 {
        level_0_spawner.must_spawn(rc_serial_read(r.uart_0, uart_cfg));
    } else {
        defmt::error!("{}: No USART1 config found", ID);
    }

    if let Some(motor_cfg) = &config.motors {
        level_0_spawner.must_spawn(motor_governor(r.motors, motor_cfg));
    } else {
        defmt::error!("{}: No motor protocol config found", ID)
    }

    level_0_spawner.must_spawn(common::tasks::signal_router::main());
    level_0_spawner.must_spawn(common::tasks::imu_manager::main());
    level_0_spawner.must_spawn(common::tasks::rate_loop::main());

    // ----------------- medium-priority tasks ------------------
    
    if let Some(uart_cfg) = &config.uart2 {
        level_1_spawner.must_spawn(gnss_serial_read(r.uart_1, uart_cfg));
    } else {
        defmt::error!("{}: No USART2 config found", ID);
    }

    level_1_spawner.must_spawn(common::tasks::commander::main());
    level_1_spawner.must_spawn(common::tasks::att_estimator::main());
    level_1_spawner.must_spawn(common::tasks::angle_loop::main());

    // ------------------- Low-priority tasks -------------------

    // TODO 
    // if let Some(sdmmc_cfg) = &config.sdmmc {
    //     level_t_spawner.must_spawn(blackbox_fat(r.sdcard, sdmmc_cfg));
    // } else {
    //     defmt::error!("{}: No SDMMC config found", ID);
    // }

    // A valid config always exists for the usb-handler otherwise
    // the user will not be able to interface with the system.
    level_t_spawner.must_spawn(usb_handler(r.usb, &config.info));

    level_t_spawner.must_spawn(common::tasks::calibrator::main());
    level_t_spawner.must_spawn(common::tasks::arm_blocker::main());
    level_t_spawner.must_spawn(common::tasks::signal_stats::main());

    // -------------------------- fin ---------------------------

    defmt::info!("{}: setup finished!", ID);
}
