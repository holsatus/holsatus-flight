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
pub mod metrics;
pub mod plot;
pub mod resources;

#[cfg(feature = "rerun")]
pub mod rerun_logger;

use crate::metrics::{MissionPhase, MissionReport, Sample};

pub static RUNTIME: LazyLock<Runtime> = LazyLock::new(|| {
    let runtime = Runtime::new().expect("Unable to create tokio Runtime");
    Box::leak(Box::new(runtime.enter()));
    runtime
});

static RUNNING: AtomicBool = AtomicBool::new(true);

/// Ground-truth position from sim physics (NED: negative z = up).
/// Written by the sim loop, read by hover mission (alt) and position hold (xy).
static SIM_TRUE_POS: [std::sync::atomic::AtomicI32; 3] = [
    std::sync::atomic::AtomicI32::new(0),
    std::sync::atomic::AtomicI32::new(0),
    std::sync::atomic::AtomicI32::new(0),
];

fn set_sim_pos(x: f32, y: f32, z: f32) {
    SIM_TRUE_POS[0].store((x * 1000.0) as i32, Ordering::Relaxed);
    SIM_TRUE_POS[1].store((y * 1000.0) as i32, Ordering::Relaxed);
    SIM_TRUE_POS[2].store((z * 1000.0) as i32, Ordering::Relaxed);
}
fn get_sim_pos() -> [f32; 3] {
    [
        SIM_TRUE_POS[0].load(Ordering::Relaxed) as f32 / 1000.0,
        SIM_TRUE_POS[1].load(Ordering::Relaxed) as f32 / 1000.0,
        SIM_TRUE_POS[2].load(Ordering::Relaxed) as f32 / 1000.0,
    ]
}
fn get_sim_alt() -> f32 { get_sim_pos()[2] }

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
    let _ = env_logger::try_init();
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

// ------------------------------------------------------------------
// H743v2 hover test: matches the PID gains and mission from flight.rs
// ------------------------------------------------------------------

pub fn hover_test_entry(
    limit_seconds: u64,
    #[allow(unused)] test_name: &str,
) -> Result<MissionReport, Box<dyn std::error::Error>> {
    let _enter = RUNTIME.enter();

    // Reset the shared phase marker: a previous test in the same process may
    // have left it in a non-zero state.
    crate::metrics::set_phase(MissionPhase::Preflight);

    let run_inputs = crate::metrics::get_run_inputs();
    let sim_config_path = run_inputs.sim_config.clone();
    let mut config = holsatus_sim::config::load_from_file_path(&sim_config_path)?;
    // Allow the sweep harness to override the end-to-end sensor-to-actuator
    // pipeline delay without generating a separate sim-config file per value.
    config.vehicle.pipeline_latency_steps = run_inputs.pipeline_latency_steps;
    let (r, sitl) = holsatus_sim::initialize(config.clone())?;

    #[cfg(feature = "rerun")]
    let mut logger = rerun_logger::setup(sitl.clone(), 10, test_name)?;
    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    let fw_sitl = sitl.clone();
    let report = std::sync::Arc::new(std::sync::Mutex::new(MissionReport::default()));
    let report_writer = report.clone();
    let mut subsample = 0u32;
    // t0 is captured on the first step: Instant::now() returns a sentinel
    // u64::MAX before lockstep_with starts the driver.
    let mut t0: Option<Instant> = None;

    lockstep::lockstep_with(
        move |spawner| hover_firmware_entry(spawner, r, fw_sitl),
        move || {
            #[cfg(feature = "rerun")]
            logger.log_subsampled().unwrap();

            assert!(Instant::now().as_secs() < limit_seconds);

            let step_size = embassy_time::Duration::from_hz(SIM_FREQUENCY);
            sitl.step(step_size.as_micros() as f32 * 1e-6);

            let state = <SimHandle as Sim>::vehicle_state(&sitl);
            set_sim_pos(state.position.x, state.position.y, state.position.z);

            // Record full ground-truth sample at 50 Hz (every 10th step at 500 Hz).
            subsample += 1;
            if subsample % 10 == 0 {
                let (roll, pitch, yaw) = state.rotation.euler_angles();
                // Angle between body-z axis and world-z axis: element (2,2) of
                // the rotation matrix is cos(tilt). Both axes are "down" in NED,
                // so a level drone gives cos_tilt = 1 (tilt = 0 rad).
                let rot_mat = state.rotation.to_rotation_matrix();
                let cos_tilt = rot_mat.matrix()[(2, 2)].clamp(-1.0, 1.0);
                let tilt = cos_tilt.acos();
                let now = Instant::now();
                let t0_instant = *t0.get_or_insert(now);
                let dt_s = (now - t0_instant).as_millis() as f32 / 1000.0;
                report_writer.lock().unwrap().push(Sample {
                    t: dt_s,
                    phase: crate::metrics::get_phase(),
                    pos: [state.position.x, state.position.y, state.position.z],
                    roll,
                    pitch,
                    yaw,
                    tilt_from_vertical: tilt,
                });
            }

            RUNNING.load(Ordering::Relaxed).then_some(step_size)
        },
    );

    #[cfg(feature = "rerun")]
    std::thread::sleep(std::time::Duration::from_millis(100));

    let out = std::sync::Arc::try_unwrap(report).unwrap().into_inner().unwrap();
    Ok(out)
}

fn hover_firmware_entry(spawner: Spawner, r: Resources, _sim: SimHandle) {
    use std::sync::atomic::Ordering;

    log::info!("H743v2 hover firmware entry");

    common::signals::CONTROL_FREQUENCY.store(SIM_FREQUENCY as u16, Ordering::Relaxed);

    // IMPORTANT: spawn param_storage first, then the setup task which
    // overrides PID gains and THEN spawns the controller tasks. The
    // controllers create PIDs from table params at startup and never
    // reload, so params must be set before they read.
    spawner.spawn(resources::param_storage(r.flash).unwrap());
    spawner.spawn(h743v2_setup_and_run(spawner, r.imu, r.motors).unwrap());
    spawner.spawn(h743v2_hover_mission().unwrap());
    spawner.spawn(h743v2_ahrs_to_eskf_bridge().unwrap());
    spawner.spawn(h743v2_angle_to_rate_bridge().unwrap());
    spawner.spawn(h743v2_flow_hold().unwrap());
}

/// Set PID gains, motor governor timeout, then spawn controller tasks.
/// Must run BEFORE controllers read their params tables.
#[embassy_executor::task]
async fn h743v2_setup_and_run(
    spawner: Spawner,
    imu: holsatus_sim::SimulatedImu,
    motors: holsatus_sim::SimulatedMotors,
) {
    use common::tasks::controller_rate;
    use common::tasks::controller_angle;
    use common::tasks::motor_governor;

    // Wait for param_storage to initialize table defaults from flash.
    Timer::after_millis(10).await;

    // Apply rate + angle gains from the shared RunInputs (defaults baked into
    // RunInputs::default match the previous hardcoded values).
    let inputs = crate::metrics::get_run_inputs();
    {
        let mut r = controller_rate::params::TABLE.params.write().await;
        r.x.kp = inputs.rate_roll.kp;
        r.x.ki = inputs.rate_roll.ki;
        r.x.kd = inputs.rate_roll.kd;
        r.y.kp = inputs.rate_pitch.kp;
        r.y.ki = inputs.rate_pitch.ki;
        r.y.kd = inputs.rate_pitch.kd;
        r.z.kp = inputs.rate_yaw.kp;
        r.z.ki = inputs.rate_yaw.ki;
        r.z.kd = inputs.rate_yaw.kd;
    }

    {
        let mut a = controller_angle::params::TABLE.params.write().await;
        a.roll.ki = inputs.angle_roll_ki;
        a.pitch.ki = inputs.angle_pitch_ki;
    }

    // Motor governor: match flight.rs 500ms timeout
    {
        let mut m = motor_governor::params::TABLE.params.write().await;
        m.timeout_ms = 500;
    }

    log::info!("H743v2 params set, spawning controllers");

    // NOW spawn the controllers -- they will read the overridden params.
    spawner.spawn(resources::imu_reader(imu).unwrap());
    spawner.spawn(resources::motor_governor(motors).unwrap());
    spawner.spawn(common::tasks::controller_rate::main().unwrap());
    spawner.spawn(common::tasks::att_estimator::main().unwrap());
    spawner.spawn(common::tasks::controller_angle::main().unwrap());

    // Runtime IMU calibration: average samples to remove sensor bias.
    // flight.rs uses `mean_z - GRAVITY` for Z, which assumes Z-up.
    // The sim outputs NED (Z-down), so we only calibrate X/Y accel and
    // all 3 gyro axes to avoid flipping the Z convention.
    {
        use common::tasks::imu_reader;
        const N: u32 = 500;
        let mut rcv = common::signals::RAW_MULTI_IMU_DATA[0].receiver();
        rcv.changed().await;
        let mut sum_acc = [0.0_f64; 3];
        let mut sum_gyr = [0.0_f64; 3];
        for _ in 0..N {
            let d = rcv.changed().await;
            for i in 0..3 {
                sum_acc[i] += d.acc[i] as f64;
                sum_gyr[i] += d.gyr[i] as f64;
            }
        }
        let n = N as f64;
        // Only X/Y accel bias -- Z left at 0 to preserve NED convention.
        let acc_bias = [
            (sum_acc[0] / n) as f32,
            (sum_acc[1] / n) as f32,
            0.0,
        ];
        let gyr_bias = [
            (sum_gyr[0] / n) as f32,
            (sum_gyr[1] / n) as f32,
            (sum_gyr[2] / n) as f32,
        ];
        {
            let mut p = imu_reader::params::TABLE.params.write().await;
            p.cal_acc.bias = acc_bias;
            p.cal_gyr.bias = gyr_bias;
        }
        imu_reader::CHANNEL[0]
            .sender()
            .send(imu_reader::Message::ReloadParams)
            .await;
        log::info!("IMU cal: acc_bias=[{:.3},{:.3},{:.3}] gyr_bias=[{:.4},{:.4},{:.4}]",
            acc_bias[0], acc_bias[1], acc_bias[2],
            gyr_bias[0], gyr_bias[1], gyr_bias[2]);
    }
}

/// Mirrors the ahrs_to_eskf_bridge from flight.rs
#[embassy_executor::task]
async fn h743v2_ahrs_to_eskf_bridge() {
    use common::nalgebra::Vector3;
    use common::tasks::eskf::EskfEstimate;

    let mut rcv = common::signals::AHRS_ATTITUDE_Q.receiver();
    let mut snd = common::signals::ESKF_ESTIMATE.sender();
    loop {
        let att = rcv.changed().await;
        snd.send(EskfEstimate {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            att,
            gyr_bias: Vector3::zeros(),
            acc_bias: Vector3::zeros(),
        });
    }
}

/// Bridge angle controller output to rate setpoint.
/// Unlike flight.rs which zeroes yaw (no compass), the sim has a clean
/// gyro so we pass yaw through to prevent the slow yaw drift that causes
/// the helix pattern.
#[embassy_executor::task]
async fn h743v2_angle_to_rate_bridge() {
    let mut rcv = common::signals::ANGLE_TO_RATE_SP.receiver();
    let mut snd = common::signals::TRUE_RATE_SP.sender();
    loop {
        let [roll, pitch, yaw] = rcv.changed().await;
        snd.send([roll, pitch, yaw]);
    }
}

/// Simulated MTF-01 optical flow sensor + position hold controller.
///
/// Models the real MTF-01 signal chain: ground-truth velocity is converted
/// to integer motion counts with realistic noise (D000302 profile), then
/// back to velocity. The velocity is integrated to estimate position, and
/// a PD controller on position + velocity commands attitude tilts.
///
/// This goes beyond flight.rs's velocity-only damping: integrating flow
/// into position gives actual station-keeping, which is needed to stay
/// within 10cm std dev. The same approach can be ported to the real drone.
#[embassy_executor::task]
async fn h743v2_flow_hold() {
    use common::nalgebra::UnitQuaternion;
    use rand::SeedableRng;

    let mut snd = common::signals::TRUE_ATTITUDE_Q_SP.sender();

    // Pull MTF-01 + flow-hold parameters from RunInputs so they appear in the
    // test report (and so a sweep can vary them).
    let inputs = crate::metrics::get_run_inputs();
    let flow_scale = inputs.flow_scale;
    let flow_noise_sigma = inputs.flow_noise_sigma;
    let kp_pos = inputs.flow_kp_pos;
    let kd_vel = inputs.flow_kd_vel;
    let max_tilt_rad = inputs.flow_max_tilt_rad;

    // Seeded RNG for the MTF-01 noise model: same seed + same inputs produces
    // the same sample stream, making pass/fail reproducible and sweep boundaries
    // trustworthy.
    let mut rng = rand::rngs::StdRng::seed_from_u64(inputs.flow_noise_seed);
    let noise_dist = rand_distr::Normal::new(0.0_f32, flow_noise_sigma).unwrap();

    const DT: f32 = 0.02; // 50 Hz

    // Integrated position estimate from flow (starts at origin).
    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;

    // Wait until motors are armed (same as hover mission).
    {
        use common::types::actuators::MotorsState;
        let mut rcv = common::signals::MOTORS_STATE.receiver();
        loop {
            match rcv.changed().await {
                MotorsState::ArmedIdle | MotorsState::Armed(_) => break,
                _ => {}
            }
        }
    }
    log::info!("Flow hold active");

    // Initialize prev position to current sim position.
    let [init_x, init_y, _] = get_sim_pos();
    let mut prev_x = init_x;
    let mut prev_y = init_y;

    loop {
        let [x, y, z] = get_sim_pos();
        let height = (-z).max(0.05);

        // World-frame velocity from position finite difference.
        let world_vx = (x - prev_x) / DT;
        let world_vy = (y - prev_y) / DT;
        prev_x = x;
        prev_y = y;

        // The real MTF-01 measures body-frame velocity, not world-frame.
        // Rotate world velocity into body frame using the current attitude
        // estimate (yaw component). This matters because Madgwick yaw drifts
        // without a magnetometer.
        let yaw = match common::signals::AHRS_ATTITUDE_Q.receiver().try_get() {
            Some(q) => q.euler_angles().2,
            None => 0.0,
        };
        let (sy, cy) = yaw.sin_cos();
        let body_vx =  world_vx * cy + world_vy * sy;
        let body_vy = -world_vx * sy + world_vy * cy;

        // Simulate MTF-01: body velocity -> sensor motion counts + noise -> velocity.
        // MTF-01 axis convention (from flight.rs mtf01_reader):
        //   vx (forward) = motion_y * flow_scale * h
        //   vy (right)   = -motion_x * flow_scale * h
        // Invert to get motion counts from body velocity:
        let true_motion_y = body_vx / (flow_scale * height);
        let true_motion_x = -body_vy / (flow_scale * height);

        let noisy_motion_x = (true_motion_x + rand_distr::Distribution::sample(&noise_dist, &mut rng)).round();
        let noisy_motion_y = (true_motion_y + rand_distr::Distribution::sample(&noise_dist, &mut rng)).round();

        // Convert back to velocity (same formula as mtf01_reader).
        let flow_vx = noisy_motion_y * flow_scale * height;
        let flow_vy = -noisy_motion_x * flow_scale * height;

        // -- Safeguard 1: spike rejection. Skip samples with unrealistic
        // velocity (> 1.0 m/s from a single flow frame on a hovering drone).
        // Prevents corrupted flow readings from poisoning the position estimate.
        const MAX_FLOW_VEL: f32 = 1.0;
        if flow_vx.abs() > MAX_FLOW_VEL || flow_vy.abs() > MAX_FLOW_VEL {
            Timer::after_millis(20).await;
            continue;
        }

        // Rotate body-frame flow velocity back to world frame for position integration.
        let world_flow_vx = flow_vx * cy - flow_vy * sy;
        let world_flow_vy = flow_vx * sy + flow_vy * cy;

        // Integrate in world frame (dead reckoning).
        est_x += world_flow_vx * DT;
        est_y += world_flow_vy * DT;

        // -- Safeguard 2: position estimate clamp. If the estimate drifts
        // beyond 0.5m, freeze it and fall back to velocity-only damping.
        // Prevents the controller from driving the drone into a wall
        // if the position integration goes wrong.
        const POS_CLAMP: f32 = 0.5;
        let pos_ok = est_x.abs() < POS_CLAMP && est_y.abs() < POS_CLAMP;
        est_x = est_x.clamp(-POS_CLAMP, POS_CLAMP);
        est_y = est_y.clamp(-POS_CLAMP, POS_CLAMP);

        // PD position hold: compute world-frame position error, then rotate
        // to body frame for the tilt commands.
        let body_err_x =  est_x * cy + est_y * sy;
        let body_err_y = -est_x * sy + est_y * cy;

        // When position is clamped (drifted too far), use only velocity damping
        // with reduced gain so the drone slows down but doesn't chase a wrong target.
        let kp = if pos_ok { kp_pos } else { 0.0 };

        // Pitch: positive body_err_x = drone ahead of target -> nose up (positive pitch)
        //   to brake and return. Matches flight.rs: pitch_tilt = +KP * vx.
        // Roll: positive body_err_y = drone right of target -> roll left (negative roll).
        //   Matches flight.rs: roll_tilt = -KP * vy.
        let pitch_cmd = ( (kp * body_err_x + kd_vel * flow_vx)).clamp(-max_tilt_rad, max_tilt_rad);
        let roll_cmd  = (-(kp * body_err_y + kd_vel * flow_vy)).clamp(-max_tilt_rad, max_tilt_rad);

        snd.send(UnitQuaternion::from_euler_angles(roll_cmd, pitch_cmd, 0.0));
        Timer::after_millis(20).await;
    }
}

/// Ground-start hover mission with slow thrust ramp.
///
/// Phase 0 (RAMP): thrust ramps linearly from 0 to hover over RAMP_S seconds.
///   Liftoff occurs naturally at ~50% when thrust = weight (around second 5).
///   This gives 5 seconds of "spinning but not flying" to spot problems.
/// Phase 1 (CLIMB): once airborne, PID altitude controller ramps setpoint to 1m.
/// Phase 2 (HOVER): hold at 1m.
/// Phase 3 (DESCEND): ramp thrust back down to land.
#[embassy_executor::task]
async fn h743v2_hover_mission() {
    use common::nalgebra::UnitQuaternion;
    use common::types::actuators::MotorsState;

    Timer::after_secs(1).await;

    common::signals::TRUE_RATE_SP.send([0.0_f32; 3]);
    common::signals::TRUE_Z_THRUST_SP.send(0.0_f32);
    common::signals::TRUE_ATTITUDE_Q_SP.send(UnitQuaternion::identity());

    Timer::after_millis(500).await;

    log::info!("Arming motors");
    common::tasks::commander::COMMAD_ARM_VEHICLE.send(true);

    let mut rcv = common::signals::MOTORS_STATE.receiver();
    loop {
        match rcv.changed().await {
            MotorsState::ArmedIdle | MotorsState::Armed(_) => break,
            _ => {}
        }
    }
    log::info!("Motors armed");

    const MASS: f32 = 0.18;
    const GRAVITY: f32 = 9.81;
    const HOVER_THRUST: f32 = MASS * GRAVITY; // 1.766 N
    const DT: f32 = 0.02;

    // Read mission + altitude parameters from RunInputs so the report and any
    // sweep harness see the same config the mission actually used.
    let inputs = crate::metrics::get_run_inputs();
    let ramp_s = inputs.mission_ramp_s;
    let climb_s = inputs.mission_climb_s;
    let hover_s = inputs.mission_hover_s;
    let descend_s = inputs.mission_descend_s;
    let target_alt = -inputs.target_altitude_m; // NED: negative z = up
    let alt_kp = inputs.alt.kp;
    let alt_ki = inputs.alt.ki;
    let alt_kd = inputs.alt.kd;
    let int_clamp = inputs.alt_int_clamp;
    let max_thrust = HOVER_THRUST * inputs.alt_max_thrust_frac;
    let min_thrust = HOVER_THRUST * inputs.alt_min_thrust_frac;

    // Phase 0: slow thrust ramp from 0 to HOVER_THRUST over ramp_s seconds.
    // Liftoff at ~ramp_s/2 when thrust crosses weight.
    log::info!("P0: thrust ramp ({}s, liftoff ~{}s)", ramp_s, ramp_s / 2);
    crate::metrics::set_phase(MissionPhase::Ramp);
    let ramp_start = embassy_time::Instant::now();
    let mut log_div = 0u32;
    loop {
        let elapsed = ramp_start.elapsed().as_millis();
        if elapsed >= ramp_s * 1000 { break; }
        let frac = elapsed as f32 / (ramp_s * 1000) as f32;
        let thr = HOVER_THRUST * frac;
        common::signals::TRUE_Z_THRUST_SP.send(thr);
        log_div += 1;
        if log_div % 25 == 0 {
            let z = get_sim_alt();
            eprintln!("ramp: t={:.1}s thr={:.3} z={:.3}", elapsed as f32 / 1000.0, thr, z);
        }
        Timer::after_millis(20).await;
    }

    let mut integral = 0.0_f32;
    let mut prev_z = get_sim_alt();

    let alt_pid = |sp: f32, integral: &mut f32, prev_z: &mut f32, div: &mut u32| -> f32 {
        let actual = get_sim_alt();
        let err = actual - sp;
        let vel_z = (actual - *prev_z) / DT;
        *prev_z = actual;
        *integral = (*integral + err * DT).clamp(-int_clamp, int_clamp);
        let thr = (HOVER_THRUST + alt_kp * err + alt_ki * *integral + alt_kd * vel_z)
            .clamp(min_thrust, max_thrust);
        *div += 1;
        if *div % 25 == 0 {
            eprintln!("alt: z={:.3} sp={:.3} vz={:.2} thr={:.3}", actual, sp, vel_z, thr);
        }
        thr
    };

    log::info!("P1: climb to 1m ({}s)", climb_s);
    crate::metrics::set_phase(MissionPhase::Climb);
    let start = embassy_time::Instant::now();
    loop {
        let elapsed = start.elapsed().as_millis();
        if elapsed >= climb_s * 1000 { break; }
        let sp = target_alt * (elapsed as f32 / (climb_s * 1000) as f32);
        common::signals::TRUE_Z_THRUST_SP.send(alt_pid(sp, &mut integral, &mut prev_z, &mut log_div));
        Timer::after_millis(20).await;
    }

    log::info!("P2: hover ({}s)", hover_s);
    crate::metrics::set_phase(MissionPhase::Hover);
    let hover_start = embassy_time::Instant::now();
    loop {
        let elapsed = hover_start.elapsed().as_millis();
        if elapsed >= hover_s * 1000 { break; }
        common::signals::TRUE_Z_THRUST_SP.send(alt_pid(target_alt, &mut integral, &mut prev_z, &mut log_div));
        Timer::after_millis(20).await;
    }

    // Phase 3: PID-controlled descent (ramp setpoint back to ground).
    log::info!("P3: descend ({}s)", descend_s);
    crate::metrics::set_phase(MissionPhase::Descend);
    let desc_start = embassy_time::Instant::now();
    loop {
        let elapsed = desc_start.elapsed().as_millis();
        if elapsed >= descend_s * 1000 { break; }
        let sp = target_alt * (1.0 - elapsed as f32 / (descend_s * 1000) as f32);
        common::signals::TRUE_Z_THRUST_SP.send(alt_pid(sp, &mut integral, &mut prev_z, &mut log_div));
        Timer::after_millis(20).await;
    }
    common::signals::TRUE_Z_THRUST_SP.send(0.0);

    Timer::after_secs(2).await;
    crate::metrics::set_phase(MissionPhase::Landed);
    log::info!("Disarming");
    common::tasks::commander::COMMAD_ARM_VEHICLE.send(false);
    Timer::after_secs(1).await;
    RUNNING.store(false, Ordering::Relaxed);
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
