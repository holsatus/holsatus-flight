use std::{sync::atomic::Ordering, time::Duration};

use clap::Parser;
use common::{signals::ESKF_ESTIMATE, types::control::RcAnalog};

mod rerun_logger;
mod resources;
mod thread_executor;
mod ticker;

#[derive(clap::Parser)]
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
    resources::simulation_runner(sitl.clone(), 1000);
    resources::new_tcp_serial_io("127.0.0.1:15001", "tcp-serial1");
    resources::run_simulated_vicon(sitl.clone());

    // Might as well start the parameter storage module to get things loaded
    level_t_spawner.spawn(resources::param_storage(r.flash).unwrap());

    // ------------------ high-priority tasks -------------------

    // These take direct ownership of their hardware to avoid additional complexity
    level_0_spawner.spawn(resources::imu_reader(r.imu).unwrap());
    level_0_spawner.spawn(resources::motor_governor(r.motors).unwrap());

    // level_0_spawner.spawn(common::tasks::rc_reader::main("serial0").unwrap()); // TODO Emulate serial?
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

    thread_executor::RUNTIME.spawn(autonomous_flight_intercept());

    // spawner_1.spawn(resources::gnss_reader(r.vicon).unwrap());

    // Park the current thread
    std::thread::park();
    Ok(())
}

fn millis_in_the_future(millis: u64) -> common::embassy_time::Instant {
    let now = common::embassy_time::Instant::now();
    now + common::embassy_time::Duration::from_millis(millis)
}

async fn autonomous_flight_basic() {
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
    rcv_motors_state.get_and(|state|state.is_armed()).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -5.0],
            millis_in_the_future(2000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    tokio::time::sleep(Duration::from_secs(10)).await;

    // Get down to ground level
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -5.0],
            millis_in_the_future(2000),
        ))
        .await;

    // Wait for us to be close enough
    receiver.get_and(|est| est.pos[2] > -5.2).await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, 0.5],
            millis_in_the_future(1000),
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


async fn autonomous_flight_spacex() {
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
    rcv_motors_state.get_and(|state|state.is_armed()).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -1.0],
            millis_in_the_future(1000),
        ))
        .await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [10.0, 0.0, -5.0],
            millis_in_the_future(4000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    tokio::time::sleep(Duration::from_secs(2)).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [-10.0, 0.0, -5.0],
            millis_in_the_future(4000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    tokio::time::sleep(Duration::from_secs(2)).await;


    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [10.0, 0.0, -5.0],
            millis_in_the_future(4000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    tokio::time::sleep(Duration::from_secs(2)).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [-10.0, 0.0, -5.0],
            millis_in_the_future(4000),
        ))
        .await;

    // Hover at 5 meters for 5 seconds
    tokio::time::sleep(Duration::from_secs(5)).await;


    // Get down to ground level
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [20.0, 0.0, -25.0],
            millis_in_the_future(2000),
        ))
        .await;

    // Wait for us to be close enough
    receiver.get_and(|est| est.pos[2] > -25.2 && est.pos[0].abs() < 0.1 && est.pos[1].abs() < 0.1).await;
    
    tokio::time::sleep(Duration::from_secs(5)).await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -0.5],
            millis_in_the_future(1000),
        ))
        .await;

    // Wait for us to almost contact the ground
    receiver.get_and(|est| est.pos[2] > -0.6).await;

    // Get all the way to the ground
    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, 0.0],
            millis_in_the_future(500),
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


async fn autonomous_flight_intercept() {
    use common::tasks::commander::*;

    log::warn!("Sending arming command");
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

    log::warn!("Sending control mode command");
    PROCEDURE
        .send(Request {
            command: message::SetControlMode::Autonomous.into(),
            origin: Origin::Automatic,
        })
        .await;

    let mut rcv_motors_state = common::signals::MOTORS_STATE.receiver();
    rcv_motors_state.get_and(|state|state.is_armed()).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [2.0, 0.0, -5.0],
            common::embassy_time::Instant::now(),
        ))
        .await;

    tokio::time::sleep(Duration::from_secs(10)).await;

    fn millis_in_the_future(millis: u64) -> common::embassy_time::Instant {
        let now = common::embassy_time::Instant::now();
        now + common::embassy_time::Duration::from_millis(millis)
    }

    for i in 0..200 {
        common::tasks::controller_mpc::CHANNEL
            .send(common::tasks::controller_mpc::Message::SetPositionAt(
                [100.0 - i as f32 * 1.0 , 10.0, -50.0],
                millis_in_the_future(9900),
            ))
            .await;

        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    tokio::time::sleep(Duration::from_millis(10000)).await;
    receiver.get_and(|est| est.vel[0].abs() < 1.0).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, -5.0],
            millis_in_the_future(4000),
        ))
        .await;

    receiver.get_and(|est| est.vel[0].abs() < 1.0).await;
    receiver.get_and(|est| est.pos[2] > -5.2).await;

    common::tasks::controller_mpc::CHANNEL
        .send(common::tasks::controller_mpc::Message::SetPositionAt(
            [0.0, 0.0, 0.0],
            millis_in_the_future(1000),
        ))
        .await;

    receiver.get_and(|est| est.pos[2] > -0.1).await;


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

async fn step_responses() {
    use common::tasks::commander::*;

    log::warn!("Sending arming command");
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

    log::warn!("Sending control mode command");
    PROCEDURE
        .send(Request {
            command: message::SetControlMode::Rate.into(),
            origin: Origin::Automatic,
        })
        .await;


    let mut rcv_motors_state = common::signals::MOTORS_STATE.receiver();
    rcv_motors_state.get_and(|state|state.is_armed()).await;

    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 10.0));
    tokio::time::sleep(Duration::from_secs(2)).await;

    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 1.0));
    tokio::time::sleep(Duration::from_millis(1200)).await;

    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 6.4));
    tokio::time::sleep(Duration::from_millis(500)).await;

    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(1.2, 0.0, 0.0, 6.4));
    tokio::time::sleep(Duration::from_secs(1)).await;

    PROCEDURE
    .send(Request {
        command: message::SetControlMode::Angle.into(),
        origin: Origin::Automatic,
    })
    .await;

    tokio::time::sleep(Duration::from_millis(10)).await;

    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 6.0));
    tokio::time::sleep(Duration::from_millis(500)).await;
    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 25.0));
    tokio::time::sleep(Duration::from_millis(400)).await;
    common::signals::RC_ANALOG_UNIT.send(RcAnalog::new(0.0, 0.0, 0.0, 6.4));

    
}
