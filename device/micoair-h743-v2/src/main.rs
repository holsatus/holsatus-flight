#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

mod config;
mod resources;
use resources::{flash, motors, spi, usart};

#[embassy_executor::main]
async fn main(level_t_spawner: embassy_executor::Spawner) {
    // ---------------------- early setup -----------------------

    // Initialize the chip and split the resources
    let c = stm32_support::default_config();
    let p = embassy_stm32::init(c);
    let r = resources::split(p);
    common::embassy_time::Timer::after_millis(10).await;

    defmt::info!("[stm32f405-dev] clocks initialized, starting tasks");

    // The native sampling time of the ICM20948
    common::signals::CONTROL_FREQUENCY.store(1125, Ordering::Relaxed);

    // Create interrupt executors
    let level_0_spawner = stm32_support::interrupt_executor!(FDCAN2_IT0, P10);
    let level_1_spawner = stm32_support::interrupt_executor!(FDCAN2_IT1, P11);

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(flash::param_storage(r.flash, config::flash()).unwrap());

    // Give special priority to the serial port used as primary input
    level_0_spawner.spawn(usart::run_usart1(r.usart_1, config::usart1(), "usart1").unwrap()); // CRSF
    level_1_spawner.spawn(usart::run_usart2(r.usart_2, config::usart2(), "usart2").unwrap()); // UNUSED
    level_1_spawner.spawn(usart::run_usart3(r.usart_3, config::usart3(), "usart3").unwrap()); // MAVLINK
    level_1_spawner.spawn(usart::run_usart6(r.usart_6, config::usart6(), "usart6").unwrap()); // GNSS

    common::embassy_time::Timer::after_millis(10).await;

    #[cfg(feature = "usb")]
    {
        // Note: We run the peripheral at a higher priority because the shell
        // parser operates on a blocking Write implementation. This way the
        // code will nenver truly block, since the write results in an interrupt.
        level_1_spawner.spawn(resources::usb::runner(r.usb, config::hwinfo()).unwrap());
        level_t_spawner.spawn(common::shell::main("usb").unwrap());
    }

    #[cfg(feature = "sdmmc")]
    level_t_spawner.spawn(resources::sdmmc::blackbox_fat(r.sdcard, config::sdmmc()).unwrap());

    common::embassy_time::Timer::after_millis(1).await;

    // ------------------ high-priority tasks -------------------

    // These take direct ownership of their hardware to avoid additional complexity
    level_0_spawner.spawn(spi::bmi088_reader(r.spi_2, r.spi_2_extra).unwrap());
    level_0_spawner.spawn(spi::bmi270_reader(r.spi_3, r.spi_3_extra).unwrap());
    // level_0_spawner.spawn(i2c::imu_reader(r.i2c_2, config::i2c1(), config::imu()).unwrap());
    level_0_spawner.spawn(motors::motor_governor(r.motors, config::motor()).unwrap());

    level_0_spawner.spawn(common::tasks::rc_reader::main("usart1").unwrap());
    level_0_spawner.spawn(common::tasks::rc_binder::main().unwrap());
    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    // #[cfg(feature = "gnss")]
    // level_1_spawner.spawn(common::tasks::gnss_reader::main("usart6").unwrap());
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    #[cfg(feature = "mavlink")]
    level_t_spawner.spawn(common::mavlink::main("usart3").unwrap());

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());

    #[cfg(feature = "mpc")]
    level_t_spawner.spawn(common::tasks::controller_mpc::main().unwrap());

    level_t_spawner.spawn(common::tasks::in_flight_estimator::main().unwrap());

    // -------------------------- fin ---------------------------
    common::embassy_time::Timer::after_secs(1).await;

    loop {
        defmt::debug!("[stm32f405-dev] thread-mode execution operating as intended");
        common::embassy_time::Timer::after_secs(10).await;
    }
}
