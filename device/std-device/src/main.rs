use std::sync::atomic::Ordering;

use clap::Parser;
use common::multicopter::flight_mode::Kind;

mod rerun_logger;
mod resources;
mod thread_executor;
mod ticker;

#[derive(Parser)]
struct Args {
    /// Path to the configuration file for the simulation
    #[clap(default_value = "sim_config.toml")]
    #[clap(short, long)]
    config: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    _ = thread_executor::RUNTIME.enter();

    // Load configuration
    let config = holsatus_sim::config::load_from_file_path(&args.config)?;
    let (r, sitl) = holsatus_sim::initialize(config.clone())?;

    // Start simulation thread and crate dummy resources
    resources::setup_logging(sitl.clone())?;

    // Create spawners for the threads
    let level_0_spawner = thread_executor::new_spawner()?;
    let level_1_spawner = thread_executor::new_spawner()?;
    let level_t_spawner = thread_executor::new_spawner()?;

    common::signals::CONTROL_FREQUENCY.store(1000, Ordering::Relaxed);

    // Launch the simulation thread and MAVLink TCP runner
    resources::simulation_runner(sitl.clone(), 2000);
    resources::new_tcp_serial_io("127.0.0.1:14550", "tcp-serial1");
    resources::run_simulated_vicon(sitl.clone());

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(resources::param_storage(r.flash).unwrap());

    // ------------------ high-priority tasks -------------------

    // These take direct ownership of their hardware to avoid additional complexity
    level_0_spawner.spawn(resources::imu_reader(r.imu).unwrap());
    level_0_spawner.spawn(resources::motor_governor(r.motors).unwrap());

    // level_0_spawner.spawn(common::tasks::rc_reader::main("serial0").unwrap()); // TODO Emulate rc serial?
    level_0_spawner.spawn(common::tasks::rc_binder::main().unwrap());
    level_0_spawner.spawn(common::multicopter::attitude_control::main().unwrap());
    level_0_spawner.spawn(simulated_rc().unwrap());

    // ----------------- medium-priority tasks ------------------

    // level_1_spawner.spawn(common::tasks::gnss_reader::main("serial2").unwrap()); // TODO Emulate GNSS?
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::att_estimator::main().unwrap());
    level_1_spawner.spawn(common::multicopter::flight_mode::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());
    level_t_spawner.spawn(common::mavlink::main("tcp-serial1").unwrap());

    level_t_spawner.spawn(hover_hold_demo().unwrap());

    // Park the current thread
    std::thread::park();
    Ok(())
}

/// Emulates an RC transmitter publishing a fixed set of controls at 100 Hz.
#[embassy_executor::task]
async fn simulated_rc() -> ! {
    use common::types::control::RcAnalog;

    // Throttle slightly above hover; sticks centered.
    let rc = RcAnalog::new(0.0, 0.0, 0.0, 0.44);
    let mut ticker =
        common::embassy_time::Ticker::every(common::embassy_time::Duration::from_hz(100));
    loop {
        common::signals::RC_ANALOG_UNIT.send(rc);
        ticker.next().await;
    }
}

/* READY FOR TESTING */

#[embassy_executor::task]
async fn hover_hold_demo() {
    use common::tasks::commander::*;

    // Arm the vehicle and wait for it to be armed
    PROCEDURE
        .send(Request {
            command: Command::ArmDisarm {
                arm: true,
                force: true,
            },
            origin: Origin::Automatic,
        })
        .await;

    let mut rcv_motors_state = common::signals::MOTORS_STATE.receiver();
    rcv_motors_state.get_and(|state| state.is_armed()).await;

    // Engage stabilized (angle) mode so the vehicle levels and holds altitude.
    PROCEDURE
        .send(Request {
            command: Command::SetFlightMode(Kind::RcStabilized),
            origin: Origin::Automatic,
        })
        .await;

    let mut rcv_current_mode = common::multicopter::flight_mode::CURRENT_MODE.receiver();
    rcv_current_mode
        .get_and(|mode| *mode == common::multicopter::flight_mode::Kind::RcStabilized)
        .await;

    // Hold the hover for the duration of the simulation.
    loop {
        common::embassy_time::Timer::after_secs(1).await;
    }
}
