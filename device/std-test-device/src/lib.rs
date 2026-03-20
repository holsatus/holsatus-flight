use std::{
    f32::consts::PI,
    sync::{
        atomic::{AtomicBool, Ordering},
        LazyLock,
    },
};

use clap::Parser;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use holsatus_sim::{Resources, Sim, SimHandle};
use tokio::runtime::Runtime;

use crate::resources::simulated_vicon;

pub mod lockstep;
pub mod resources;

#[cfg(feature = "rerun")]
pub mod rerun_logger;

pub static RUNTIME: LazyLock<Runtime> = LazyLock::new(|| {
    let runtime = Runtime::new().expect("Unable to create tokio Runtime");
    Box::leak(Box::new(runtime.enter()));
    runtime
});

static RUNNING: AtomicBool = AtomicBool::new(true);

#[derive(clap::Parser)]
pub struct Args {
    /// Path to the configuration file for the simulation
    #[clap(default_value = "sim_config.toml")]
    #[clap(short, long)]
    pub config: String,
}

const SIM_FREQUENCY: u64 = 500;

pub fn test_entry(
    limit_seconds: u64,
    #[allow(unused)]
    test_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let _enter = RUNTIME.enter();

    let args = Args::parse();
    let config = holsatus_sim::config::load_from_file_path(&args.config)?;
    let (r, sitl) = holsatus_sim::initialize(config.clone())?;

    // Setup logging to run at 50 Hz
    #[cfg(feature = "rerun")]
    let mut logger = rerun_logger::setup(sitl.clone(), 10, test_name)?;

    // Sometimes rerun can take a split second to start receiving
    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    let fw_sitl = sitl.clone();
    lockstep::lockstep_with(
        move |spawner| firmware_entry(spawner, r, fw_sitl),
        move || {
            #[cfg(feature = "rerun")]
            logger.log_subsampled().unwrap();

            assert!(Instant::now().as_secs() < limit_seconds);

            let step_size = embassy_time::Duration::from_hz(SIM_FREQUENCY);
            sitl.step(step_size.as_micros() as f32 * 1e-6);
            RUNNING.load(Ordering::Relaxed).then_some(step_size)
        },
    );

    // Sometimes dropping the handle early can result in lost recs
    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    Ok(())
}

fn firmware_entry(spawner: Spawner, r: Resources, sim: SimHandle) {
    log::debug!("Firmware entry started");

    common::signals::CONTROL_FREQUENCY.store(SIM_FREQUENCY as u16, Ordering::Relaxed);

    // Might as well start the parameter storage module to get things loaded
    spawner.spawn(resources::param_storage(r.flash).unwrap());

    // ------------------ high-priority tasks -------------------

    // These take direct ownership of their hardware to avoid additional complexity
    spawner.spawn(resources::imu_reader(r.imu).unwrap());
    spawner.spawn(resources::motor_governor(r.motors).unwrap());

    spawner.spawn(common::tasks::rc_binder::main().unwrap());
    spawner.spawn(common::tasks::signal_router::main().unwrap());
    spawner.spawn(common::tasks::controller_rate::main().unwrap());

    // ----------------- medium-priority tasks ------------------

    spawner.spawn(common::tasks::commander::main().unwrap());
    spawner.spawn(common::tasks::att_estimator::main().unwrap());
    spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // ------------------- Low-priority tasks -------------------

    spawner.spawn(common::tasks::calibrator::main().unwrap());
    spawner.spawn(common::tasks::arm_blocker::main().unwrap());
    spawner.spawn(common::tasks::eskf::main().unwrap());
    spawner.spawn(common::tasks::controller_mpc::main().unwrap());

    spawner.spawn(flight_test_task().unwrap());
    spawner.spawn(simulated_vicon(sim).unwrap());
}

fn millis_in_future(millis: u64) -> common::embassy_time::Instant {
    let now = common::embassy_time::Instant::now();
    now + common::embassy_time::Duration::from_millis(millis)
}

#[embassy_executor::task]
async fn flight_test_task() {
    use common::nalgebra::SVector;
    use common::tasks::commander::*;
    use common::tasks::controller_mpc::{Message, CHANNEL};

    Timer::after_secs(1).await;

    log::warn!("Sending arming command");
    PROCEDURE
        .send(Request {
            command: Command::ArmDisarm {
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
            command: Command::SetControlMode(ControlMode::Autonomous),
            origin: Origin::Automatic,
        })
        .await;

    log::debug!("================================================");
    log::debug!("============= Starting flight test =============");
    log::debug!("================================================");

    let mut rcv_eskf_estimate = common::signals::ESKF_ESTIMATE.receiver();
    let mut rcv_motors_state = common::signals::MOTORS_STATE.receiver();
    rcv_motors_state.get_and(|state| state.is_armed()).await;

    log::info!("Stepping to 1 meter in 3 seconds");
    let position_setpoint = [0.0, 0.0, -1.0];
    CHANNEL
        .send(Message::SetPositionAt(
            position_setpoint,
            millis_in_future(3000),
        ))
        .await;

    Timer::after_secs(6).await;
    assert!(
        (rcv_eskf_estimate.get().await.pos - SVector::from(position_setpoint)).norm() < 0.5,
        "Failed to get close to target setpoint"
    );
    log::debug!("Reached setpoint: {position_setpoint:?}");

    log::info!("Stepping to 10 meters in 3 seconds");
    let position_setpoint = [0.0, 0.0, -10.0];
    CHANNEL
        .send(Message::SetPositionAt(
            position_setpoint,
            millis_in_future(3000),
        ))
        .await;

    Timer::after_secs(6).await;
    assert!(
        (rcv_eskf_estimate.get().await.pos - SVector::from(position_setpoint)).norm() < 0.5,
        "Failed to get close to target setpoint"
    );
    log::debug!("Reached setpoint: {position_setpoint:?}");

    log::info!("Initiating flight pattern");
    for i in 0..200 {
        let (sin, cos) = ((i as f32 / 40.0) * PI).sin_cos();
        let height = -((i as f32 / 20.0) * PI).cos();
        CHANNEL
            .send(Message::SetPositionAt(
                [
                    cos * 5.0,
                    (sin * 15.0).clamp(-10.0, 10.0),
                    height * 2.5 - 10.0,
                ],
                millis_in_future(5000),
            ))
            .await;

        Timer::after_millis(100).await;
    }

    log::info!("Stepping to 10 meters in 5 seconds");
    let position_setpoint = [0.0, 0.0, -10.0];
    CHANNEL
        .send(Message::SetPositionAt(
            position_setpoint,
            millis_in_future(5000),
        ))
        .await;

    Timer::after_secs(6).await;
    assert!(
        (rcv_eskf_estimate.get().await.pos - SVector::from(position_setpoint)).norm() < 0.5,
        "Failed to get close to target setpoint"
    );
    log::debug!("Reached setpoint: {position_setpoint:?}");

    log::info!("Stepping down to 2 meters in 2 seconds");
    let position_setpoint = [0.0, 0.0, -2.0];
    CHANNEL
        .send(Message::SetPositionAt(
            position_setpoint,
            millis_in_future(2000),
        ))
        .await;

    Timer::after_secs(4).await;
    assert!(
        (rcv_eskf_estimate.get().await.pos - SVector::from(position_setpoint)).norm() < 0.5,
        "Failed to get close to target setpoint"
    );
    log::debug!("Reached setpoint: {position_setpoint:?}");

    log::info!("Stepping down to 2 meters in 2 seconds");
    CHANNEL
        .send(Message::SetPositionAt(
            [0.0, 0.0, -0.2],
            millis_in_future(2000),
        ))
        .await;

    Timer::after_secs(8).await;

    log::warn!("Sending disarm command");
    PROCEDURE
        .send(Request {
            command: Command::ArmDisarm {
                arm: false,
                force: true,
            }
            .into(),
            origin: Origin::Automatic,
        })
        .await;

    Timer::after_secs(1).await;
    RUNNING.store(false, Ordering::Relaxed);
}
