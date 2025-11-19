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
    let r = resources::split(p);
    common::embassy_time::Timer::after_millis(10).await;

    defmt::info!("[stm32f405-dev] clocks initialized, starting tasks");

    // The native sampling time of the ICM20948
    common::signals::CONTROL_FREQUENCY.store(1125, Ordering::Relaxed);

    // Create interrupt executors
    let level_0_spawner = interrupt_executor!(CAN1_RX0, P10);
    let level_1_spawner = interrupt_executor!(CAN1_RX1, P11);

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(resources::param_storage(r.flash, config::flash()).unwrap());

    // Give special priority to the serial port used as primary input
    level_0_spawner.spawn(resources::run_usart1(r.usart_1, config::usart1(), "usart1").unwrap()); // CRSF
    level_1_spawner.spawn(resources::run_usart2(r.usart_2, config::usart2(), "usart2").unwrap()); // UNUSED
    level_1_spawner.spawn(resources::run_usart3(r.usart_3, config::usart3(), "usart3").unwrap()); // MAVLINK
    level_1_spawner.spawn(resources::run_usart6(r.usart_6, config::usart6(), "usart6").unwrap()); // GNSS

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
    level_0_spawner.spawn(resources::imu_reader(r.i2c_1, config::i2c1(), config::imu()).unwrap());
    level_0_spawner.spawn(resources::motor_governor(r.motors, config::motor()).unwrap());

    level_0_spawner.spawn(common::tasks::rc_reader::main("usart1").unwrap());
    level_0_spawner.spawn(common::tasks::rc_binder::main().unwrap());
    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    #[cfg(feature = "gnss")]
    level_1_spawner.spawn(common::tasks::gnss_reader::main("usart6").unwrap());
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    #[cfg(feature = "mavlink")]
    level_t_spawner.spawn(common::mavlink2::main("usart3").unwrap());

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());

    #[cfg(feature = "mpc")]
    {
        level_t_spawner.spawn(common::tasks::controller_mpc::main().unwrap());
    }

    level_t_spawner.spawn(common::tasks::in_flight_estimator::main().unwrap());
    level_t_spawner.spawn(auto_program_entry().unwrap());

    // -------------------------- fin ---------------------------
    common::embassy_time::Timer::after_secs(1).await;

    loop {
        defmt::debug!("[stm32f405-dev] thread-mode execution operating as intended");
        common::embassy_time::Timer::after_secs(10).await;
    }
}

#[embassy_executor::task]
async fn auto_program_entry() -> ! {
    use common::types::control::ControlMode;
    let mut rcv_flight_mode = common::tasks::commander::CMD_CONTROL_MODE.receiver();

    loop {
        // Wait for us change to autonomous mode, then run the auto_program
        rcv_flight_mode
            .changed_and(|mode| matches!(mode, ControlMode::Autonomous))
            .await;

        // Run the program to completion, or, until the mode changes
        common::embassy_futures::select::select(
            rcv_flight_mode.changed_and(|mode| !matches!(mode, ControlMode::Autonomous)),
            auto_program::hover_minute(),
        )
        .await;
    }
}

mod auto_program {
    use common::embassy_time::{Duration, Instant, Timer};
    use defmt::*;

    fn millis_in_future(millis: u32) -> Instant {
        Instant::now() + Duration::from_millis(millis as u64)
    }

    pub async fn side_to_side() {
        use common::tasks::commander::{message::ArmDisarm, Origin, Request, Response, PROCEDURE};
        use common::tasks::controller_mpc::{Message, CHANNEL};

        let mut motors_state = common::signals::MOTORS_STATE.receiver();
        let mut eskf_estimate = common::signals::ESKF_ESTIMATE.receiver();

        // Request the arming the vehicle, doing all checks
        let res = PROCEDURE
            .request(Request {
                command: ArmDisarm {
                    arm: true,
                    force: false,
                }
                .into(),
                origin: Origin::Automatic,
            })
            .await;

        if res.is_none_or(|res| res != Response::Accepted) {
            warn!("[auto] Arming request was not accepted");
            return;
        }

        // Wait for the arming sequence to be complete
        motors_state.changed_and(|motors| motors.is_armed()).await;

        fn millis_in_future(millis: u32) -> Instant {
            Instant::now() + Duration::from_millis(millis as u64)
        }

        // In one second we should be 25 cm in the air (NED)
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, -0.25],
                millis_in_future(1000),
            ))
            .await;
        Timer::after_secs(5).await;

        // Get up to 1 meter
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, -1.0],
                millis_in_future(1000),
            ))
            .await;
        Timer::after_secs(5).await;

        // Move 2 meters to the left
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, -2.0, -1.0],
                millis_in_future(4500),
            ))
            .await;
        Timer::after_secs(7).await;

        // Move 2 meters to the right
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 2.0, -1.0],
                millis_in_future(4500),
            ))
            .await;
        Timer::after_secs(5).await;

        // Move back to center
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, -1.0],
                millis_in_future(4500),
            ))
            .await;
        Timer::after_secs(5).await;

        // Get in for a landing
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, 0.0],
                millis_in_future(4500),
            ))
            .await;

        // Wait for the drone to be close to the ground before turning off the motors
        eskf_estimate.changed_and(|est| est.pos[2] > -0.15).await;

        // Request the arming the vehicle, doing all checks
        PROCEDURE
            .request(Request {
                command: ArmDisarm {
                    arm: false,
                    force: false,
                }
                .into(),
                origin: Origin::Automatic,
            })
            .await;
    }

    pub async fn hover_minute() {
        use common::tasks::commander::{message::ArmDisarm, Origin, Request, Response, PROCEDURE};
        use common::tasks::controller_mpc::{Message, CHANNEL};

        let mut motors_state = common::signals::MOTORS_STATE.receiver();
        let mut eskf_estimate = common::signals::ESKF_ESTIMATE.receiver();

        // Request the arming the vehicle, doing all checks
        let res = PROCEDURE
            .request(Request {
                command: ArmDisarm {
                    arm: true,
                    force: false,
                }
                .into(),
                origin: Origin::Automatic,
            })
            .await;

        if res.is_none_or(|res| res != Response::Accepted) {
            warn!("[auto] Arming request was not accepted");
            return;
        }

        // Wait for the arming sequence to be complete
        motors_state.changed_and(|motors| motors.is_armed()).await;

        // In 2 seconds we should be 5 meters cm in the air (NED)
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, -5.0],
                millis_in_future(2000),
            ))
            .await;

        // Hover for a minute
        Timer::after_secs(60).await;

        // Get in for an (approximate) landing
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, -1.0],
                millis_in_future(4500),
            ))
            .await;

        Timer::after_secs(5).await;

        // Get in for a landing
        CHANNEL
            .send(Message::SetPositionAt(
                [0.0, 0.0, 0.0],
                millis_in_future(4500),
            ))
            .await;

        // Wait for the drone to be close to the ground before turning off the motors
        eskf_estimate.changed_and(|est| est.pos[2] > -0.15).await;

        // Request arming the vehicle, doing all checks
        PROCEDURE
            .request(Request {
                command: ArmDisarm {
                    arm: false,
                    force: false,
                }
                .into(),
                origin: Origin::Automatic,
            })
            .await;
    }
}
