use std::{
    f32::consts::PI,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, LazyLock, Mutex,
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
#[clap(ignore_errors = true)]
pub struct Args {
    /// Path to the configuration file for the simulation
    #[clap(default_value = "sim_config.toml")]
    #[clap(short, long)]
    pub config: String,
}

const SIM_FREQUENCY: u64 = 500;


pub use common::types::mission_report::MissionReport;

/// Wraps `MissionReport` with simulation-specific metadata that has no
/// equivalent on real hardware.
#[derive(Clone, serde::Serialize)]
pub struct SimMissionReport {
    pub test_name: String,
    pub sim_time_s: u64,
    pub wall_time_s: f64,
    pub speed_factor: f64,
    pub total_steps: u64,
    #[serde(flatten)]
    pub report: MissionReport,
}

impl SimMissionReport {
    fn print_json(&self) {
        println!("{}", serde_json::to_string_pretty(self).unwrap());
    }

    pub fn save_to_file(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }
}

pub fn test_entry(
    limit_seconds: u64,
    test_name: &str,
) -> Result<SimMissionReport, Box<dyn std::error::Error>> {
    let _enter = RUNTIME.enter();

    RUNNING.store(true, Ordering::Relaxed);

    let args = Args::parse();
    let config = holsatus_sim::config::load_from_file_path(&args.config)?;
    let (r, sitl) = holsatus_sim::initialize(config.clone())?;

    // Setup logging to run at 50 Hz
    #[cfg(feature = "rerun")]
    let mut logger = rerun_logger::setup(sitl.clone(), 10, test_name)?;

    // When rerun is not active, fall back to env_logger (respects RUST_LOG, defaults to warn)
    #[cfg(not(feature = "rerun"))]
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .try_init();

    // Sometimes rerun can take a split second to start receiving
    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    let wall_start = std::time::Instant::now();
    let test_name = test_name.to_string();

    let mut max_altitude_m = 0.0f32;
    let mut speed_sum_ms = 0.0f32;
    let mut max_speed_ms = 0.0f32;
    let mut max_accel_ms2 = 0.0f32;
    let mut ground_collision = false;
    let mut landing_speed_ms = 0.0f32;
    let mut total_steps = 0u64;
    #[allow(unused_assignments)]
    let mut final_pos = [0.0f32; 3];

    // Shared slot so the closure can hand the report back out after lockstep ends.
    let report_slot: Arc<Mutex<Option<SimMissionReport>>> = Arc::new(Mutex::new(None));
    let report_write = report_slot.clone();

    let fw_sitl = sitl.clone();
    lockstep::lockstep_with(
        move |spawner| firmware_entry(spawner, r, fw_sitl),
        move || {
            #[cfg(feature = "rerun")]
            logger.log_subsampled().unwrap();

            // Graceful time-limit check (replaces assert)
            if Instant::now().as_secs() >= limit_seconds {
                log::warn!("Simulation time limit of {limit_seconds}s reached");
                RUNNING.store(false, Ordering::Relaxed);
            }

            let state = sitl.vehicle_state();
            let pos = state.position;

            // NED: negative z = above ground, ground is at z >= 0
            let altitude_m = -pos.z;
            let speed_ms = state.velocity.norm();

            // Specific force (gravity-subtracted), matching what body_acc_norm shows
            // in the rerun plot. Only tracked while airborne to exclude ground-contact
            // impulses from the physics solver at touchdown.
            if altitude_m > 0.3 {
                max_accel_ms2 = max_accel_ms2.max(state.body_acc.norm());
            }

            max_altitude_m = max_altitude_m.max(altitude_m);
            speed_sum_ms += speed_ms;
            max_speed_ms = max_speed_ms.max(speed_ms);

            // Ground collision: drone went below starting plane
            if pos.z > 0.1 {
                ground_collision = true;
            }

            // Landing speed: last speed recorded while near ground
            if altitude_m < 0.5 {
                landing_speed_ms = speed_ms;
            }


            total_steps += 1;
            final_pos = [pos.x, pos.y, pos.z];

            let step_size = embassy_time::Duration::from_hz(SIM_FREQUENCY);
            sitl.step(step_size.as_micros() as f32 * 1e-6);

            let still_running = RUNNING.load(Ordering::Relaxed);

            if !still_running {
                let sim_secs = Instant::now().as_secs();
                let wall_secs = wall_start.elapsed().as_secs_f64();
                let sim_report = SimMissionReport {
                    test_name: test_name.clone(),
                    sim_time_s: sim_secs,
                    wall_time_s: wall_secs,
                    speed_factor: sim_secs as f64 / wall_secs,
                    total_steps,
                    report: MissionReport {
                        mission_completed: !ground_collision,
                        max_altitude_m,
                        avg_speed_ms: speed_sum_ms / total_steps as f32,
                        max_speed_ms,
                        max_acceleration_ms2: max_accel_ms2,
                        ground_collision,
                        landing_speed_ms,
                        final_position_m: final_pos,
                    },
                };
                sim_report.print_json();
                *report_write.lock().unwrap() = Some(sim_report);
            }

            still_running.then_some(step_size)
        },
    );

    // Sometimes dropping the handle early can result in lost recs
    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    Arc::try_unwrap(report_slot)
        .map_err(|_| "report Arc still shared after lockstep")?
        .into_inner()?
        .ok_or_else(|| "simulation ended without producing a report".into())
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
