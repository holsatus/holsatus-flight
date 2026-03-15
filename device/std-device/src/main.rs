use std::sync::atomic::Ordering;

use clap::Parser;
use common::{embassy_time::Timer, signals::ESKF_ESTIMATE};

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
    level_0_spawner.spawn(common::tasks::signal_router::main().unwrap());
    level_0_spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    // level_1_spawner.spawn(common::tasks::gnss_reader::main("serial2").unwrap()); // TODO Emulate GNSS?
    level_1_spawner.spawn(common::tasks::commander::main().unwrap());
    level_1_spawner.spawn(common::tasks::att_estimator::main().unwrap());
    level_1_spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    level_t_spawner.spawn(common::tasks::calibrator::main().unwrap());
    level_t_spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    level_t_spawner.spawn(common::tasks::eskf::main().unwrap());
    level_t_spawner.spawn(common::tasks::controller_mpc::main().unwrap());
    level_t_spawner.spawn(common::mavlink2::main("tcp-serial1").unwrap());

    level_t_spawner.spawn(autonomous_flight_speedloop().unwrap());

    // Park the current thread
    std::thread::park();
    Ok(())
}

fn millis_in_future(millis: u64) -> common::embassy_time::Instant {
    let now = common::embassy_time::Instant::now();
    now + common::embassy_time::Duration::from_millis(millis)
}

/* READY FOR TESTING */

#[embassy_executor::task]
async fn autonomous_flight_speedloop() {
    use common::tasks::commander::*;

    // Arm the vehicle and wait for it to be armed
    PROCEDURE
        .send(Request {
            command: message::ArmDisarm {
                arm: true,
                force: true,
            }
            .into(),
            origin: Origin::Automatic,
        })
        .await;

    let mut receiver = ESKF_ESTIMATE.receiver();

    // Set the flight mode to autonomous to enable MPC to take control
    PROCEDURE
        .send(Request {
            command: message::SetControlMode::Autonomous.into(),
            origin: Origin::Automatic,
        })
        .await;

    let mut rcv_motors_state = common::signals::MOTORS_STATE.receiver();
    rcv_motors_state.get_and(|state| state.is_armed()).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -1.0],
            millis_in_future(1000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    Timer::after_secs(2).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -5.0],
            millis_in_future(1000),
        ))
        .await;

    Timer::after_secs(2).await;

    for i in 0..20 {
        common::tasks::controller_mpc::CHANNEL
            .send(common::tasks::controller_mpc::Message::SetPositionAt(
                [0.0 - i as f32 / 2.5, 0.0, -5.0 - i as f32],
                millis_in_future(9900),
            ))
            .await;

        Timer::after_millis(60).await;
    }

    for i in 0..8 {
        common::tasks::controller_mpc::CHANNEL
            .send(common::tasks::controller_mpc::Message::SetPositionAt(
                [-8.0 + (i*2) as f32, 0.0, -25.0],
                millis_in_future(9900),
            ))
            .await;

        Timer::after_millis(90).await;
    }

    for i in 0..20 {
        common::tasks::controller_mpc::CHANNEL
            .send(common::tasks::controller_mpc::Message::SetPositionAt(
                [8.0 - i as f32 / 2.5, 0.0, -25.0 + i as f32],
                millis_in_future(9900),
            ))
            .await;

        Timer::after_millis(60).await;
    }

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -5.0],
            millis_in_future(99000),
        ))
        .await;

    Timer::after_secs(10).await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -15.0],
            millis_in_future(1000),
        ))
        .await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -2.0],
            millis_in_future(1500),
        ))
        .await;

    // Wait for us to almost contact the ground
    receiver.get_and(|est| est.pos[2] > -2.1).await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, 0.0],
            millis_in_future(500),
        ))
        .await;

    // Wait for us to almost contact the ground
    receiver.get_and(|est| est.pos[2] > -0.1).await;

    // Disarm
    PROCEDURE
        .send(Request {
            command: message::ArmDisarm {
                arm: false,
                force: true,
            }
            .into(),
            origin: Origin::Automatic,
        })
        .await;
}
