use std::time::Duration;

use holsatus_sim::{Sim, SimHandle};
use rerun::{Arrows3D, Color, LineStrips3D, Points3D, RecordingStream, Scalars, Vec3D};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct RerunConfig {
    /// Logging frequency (Hz)
    log_rate: u16,
    /// Individual logging configurations
    log_what: WhatToLog,
    /// Path to save the rerun file
    save_path: Option<String>,
}

impl Default for RerunConfig {
    fn default() -> Self {
        Self {
            log_rate: 100,
            log_what: WhatToLog::default(),
            save_path: None,
        }
    }
}

impl Default for WhatToLog {
    fn default() -> Self {
        Self {
            acc_true: true,
            acc_sitl: true,
            grav_est: false,
            gyr_true: true,
            gyr_sitl: true,
            gyr_comp: false,
            rc_ref: true,
            rate_ref: true,
            angle_ref: false,
            slewed_sp: false,
            rate_pid_out: true,
            angle_pid_out: false,
            att_true: false,
            att_est: false,
            pos_true: true,
            pos_gnss_sitl: false,
            pos_est: false,
            vel_true: false,
            vel_gnss_sitl: false,
            vel_est: false,
            motor_commands: true,
            motor_forces: true,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct WhatToLog {
    /// Enable logging of ground-truth accelerometer data. This does not
    /// contain noise or bias, and is directly pulled from the simulation.
    acc_true: bool,
    /// Enable logging of sitl-perturbed accelerometer data. This includes noise
    /// and bias configured for the sensor.
    acc_sitl: bool,
    /// Enable logging of estimated gravity vector. This estimate is used to
    /// remove gravity from the accelerometer data when estimating attitude.
    grav_est: bool,

    /// Enable logging of ground-truth gyroscope data. This does not
    /// contain noise or bias, and is directly pulled from the simulation.
    gyr_true: bool,
    /// Enable logging of sitl-perturbed gyroscope data. This includes noise
    /// and bias configured for the sensor.
    gyr_sitl: bool,
    /// Enable logging of complementary-filtered gyroscope data.
    gyr_comp: bool,

    /// Enable logging of RC reference signals (roll, pitch, yaw thrust).
    rc_ref: bool,
    /// Enable logging of reference signals for the rate loop controller.
    rate_ref: bool,
    /// Enable logging of reference signals for the attitude loop controller.
    angle_ref: bool,
    /// Enable logging of slew-rate limited reference signals (rate and angle).
    slewed_sp: bool,

    /// Enable logging of rate loop PID controller output signals.
    rate_pid_out: bool,
    /// Enable logging of attitude loop PID controller output signals.
    angle_pid_out: bool,

    /// Enable logging of the ground-truth attitude, this does not incude
    /// noise or bias, and pulled directly from the simulation.
    att_true: bool,
    /// Enable logging of the estimated attitude, this includes any estimation
    /// error that may be present (estimated from eg. acc_sim + gyr_sim).
    att_est: bool,

    /// Enable logging of the ground-truth position, this does not incude
    /// noise or bias, and pulled directly from the simulation.
    pos_true: bool,
    /// Enable logging of the sitl-pertuebed GNSS position, this includes any
    /// noise or bias configured for the sensor.
    pos_gnss_sitl: bool,
    /// Enable logging of the estimated position, this includes any estimation
    /// error that may be present (fused from eg. pos_gnss_sim + acc_sim + gyr_sim).
    pos_est: bool,

    /// Enable logging of the ground-truth velocity, this does not incude
    /// noise or bias, and pulled directly from the simulation.
    vel_true: bool,
    /// Enable logging of the sitl-perturbed GNSS velocity, this includes any
    /// noise or bias configured for the sensor.
    vel_gnss_sitl: bool,
    /// Enable logging of the estimated velocity, this includes any estimation
    /// error that may be present (fused from eg. vel_gnss_sim + acc_sim + gyr_sim).
    vel_est: bool,

    /// Enable logging of the commanded motor speeds to the simulation.
    motor_commands: bool,
    /// Enable logging of the the forces produced by each motor. This will
    /// not necessarily have gain aligned with the `motor_commands` logs.
    motor_forces: bool,
}

pub(crate) fn rerun_thread(
    rec: RecordingStream,
    handle: SimHandle,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::collections::VecDeque;

    log::debug!("Starting rerun logging thread");

    rec.log_static("/", &rerun::ViewCoordinates::FRD())?;

    const LOG_FREQ: usize = 250;
    let mut log_ticker =
        crate::ticker::Ticker::every(Duration::from_secs_f32(1. / LOG_FREQ as f32));

    // Load 3D file for drone visualization
    let drone = rerun::Asset3D::from_file_path("fpv-drone-2.gltf")?;

    rec.log("drone", &drone)?;

    const TRAIL_LEN: usize = 1000;
    let mut pos_trail: VecDeque<Vec3D> = VecDeque::with_capacity(TRAIL_LEN);

    loop {
        log_ticker.next();

        let state = handle.vehicle_state();

        let pos = state.position;
        let rot = state.rotation;

        if pos_trail.len() >= TRAIL_LEN {
            _ = pos_trail.pop_back();
        }
        pos_trail.push_front(Vec3D(pos.into()));

        log_drone_motors(&rec, handle.clone())?;

        if let Some(imu_data) = common::signals::CAL_MULTI_IMU_DATA[0].try_get() {
            rec.log(
                "sim/firmware/acc/x",
                &Scalars::single(imu_data.acc[0] as f64),
            )?;
            rec.log(
                "sim/firmware/acc/y",
                &Scalars::single(imu_data.acc[1] as f64),
            )?;
            rec.log(
                "sim/firmware/acc/z",
                &Scalars::single(imu_data.acc[2] as f64),
            )?;

            rec.log(
                "sim/firmware/gyr/x",
                &Scalars::single(imu_data.gyr[0] as f64),
            )?;
            rec.log(
                "sim/firmware/gyr/y",
                &Scalars::single(imu_data.gyr[1] as f64),
            )?;
            rec.log(
                "sim/firmware/gyr/z",
                &Scalars::single(imu_data.gyr[2] as f64),
            )?;
        }

        if let Some(gyr_data) = common::signals::COMP_FUSE_GYR.try_get() {
            rec.log(
                "sim/firmware/gyr_comp/x",
                &Scalars::single(gyr_data[0] as f64),
            )?;
            rec.log(
                "sim/firmware/gyr_comp/y",
                &Scalars::single(gyr_data[1] as f64),
            )?;
            rec.log(
                "sim/firmware/gyr_comp/z",
                &Scalars::single(gyr_data[2] as f64),
            )?;
        }

        if let Some(estimate) = common::signals::ESKF_ESTIMATE.try_get() {
            rec.log(
                "sim/firmware/est_pos/x",
                &Scalars::single(estimate.pos[0] as f64),
            )?;
            rec.log(
                "sim/firmware/est_pos/y",
                &Scalars::single(estimate.pos[1] as f64),
            )?;
            rec.log(
                "sim/firmware/est_pos/z",
                &Scalars::single(estimate.pos[2] as f64),
            )?;

            rec.log(
                "sim/firmware/est_vel/x",
                &Scalars::single(estimate.vel[0] as f64),
            )?;
            rec.log(
                "sim/firmware/est_vel/y",
                &Scalars::single(estimate.vel[1] as f64),
            )?;
            rec.log(
                "sim/firmware/est_vel/z",
                &Scalars::single(estimate.vel[2] as f64),
            )?;

            rec.log(
                "sim/firmware/est_att/x",
                &Scalars::single(estimate.att[0] as f64),
            )?;
            rec.log(
                "sim/firmware/est_att/y",
                &Scalars::single(estimate.att[1] as f64),
            )?;
            rec.log(
                "sim/firmware/est_att/z",
                &Scalars::single(estimate.att[2] as f64),
            )?;
        }

        if let Some(mpc_reference) = common::tasks::controller_mpc::MPC_REFERENCE.try_get() {
            let reference = mpc_reference.fixed_view::<3, {common::tasks::controller_mpc::HX}>(0, 0);
            let slices = reference.column_iter().map(|col|col.clone_owned().data.0[0]);

            rec.log(
                "sim/firmware/mpc_reference_dot",
                &Points3D::new([reference.column(0).clone_owned().data.0[0]])
                    .with_radii([0.05])
            )?;

            rec.log(
                "sim/firmware/mpc_reference",
                &LineStrips3D::new([slices])
            )?;
        }

        if let Some(mpc_pos_pred) = common::tasks::controller_mpc::MPC_POS_PRED.try_get() {
            let slices = mpc_pos_pred.column_iter().map(|col|col.clone_owned().data.0[0]);

            rec.log(
                "sim/firmware/mpc_position_pred",
                &LineStrips3D::new([slices])
            )?;
        }

        if let Some(rate_sp) = common::signals::TRUE_RATE_SP.try_get() {
            rec.log(
                "sim/firmware/rate_sp/x",
                &Scalars::single(rate_sp[0] as f64),
            )?;
            rec.log(
                "sim/firmware/rate_sp/y",
                &Scalars::single(rate_sp[1] as f64),
            )?;
            rec.log(
                "sim/firmware/rate_sp/z",
                &Scalars::single(rate_sp[2] as f64),
            )?;
        }

        if let Some(attitude_q_sp) = common::signals::TRUE_ATTITUDE_Q_SP.try_get() {
            let (roll, pitch, yaw) = attitude_q_sp.euler_angles();
            rec.log(
                "sim/firmware/angl_sp/x",
                &Scalars::single(roll as f64),
            )?;
            rec.log(
                "sim/firmware/angl_sp/y",
                &Scalars::single(pitch as f64),
            )?;
            rec.log(
                "sim/firmware/angl_sp/z",
                &Scalars::single(yaw as f64),
            )?;
        }

        if let Some(rate_sp) = common::signals::SLEW_RATE_SP.try_get() {
            rec.log(
                "sim/firmware/slew_rate_sp/x",
                &Scalars::single(rate_sp[0] as f64),
            )?;
            rec.log(
                "sim/firmware/slew_rate_sp/y",
                &Scalars::single(rate_sp[1] as f64),
            )?;
            rec.log(
                "sim/firmware/slew_rate_sp/z",
                &Scalars::single(rate_sp[2] as f64),
            )?;
        }

        if let Some(rate_sp) = common::signals::FF_PRED_GYR.try_get() {
            rec.log(
                "sim/firmware/gyro_ff_pred/x",
                &Scalars::single(rate_sp[0] as f64),
            )?;
            rec.log(
                "sim/firmware/gyro_ff_pred/y",
                &Scalars::single(rate_sp[1] as f64),
            )?;
            rec.log(
                "sim/firmware/gyro_ff_pred/z",
                &Scalars::single(rate_sp[2] as f64),
            )?;
        }

        if let Some(rate_sp) = common::signals::AHRS_ATTITUDE.try_get() {
            rec.log(
                "sim/firmware/ahrs_attitude/x",
                &Scalars::single(rate_sp[0] as f64),
            )?;
            rec.log(
                "sim/firmware/ahrs_attitude/y",
                &Scalars::single(rate_sp[1] as f64),
            )?;
            rec.log(
                "sim/firmware/ahrs_attitude/z",
                &Scalars::single(rate_sp[2] as f64),
            )?;
        }

        // if let Some(MotorsState::Armed(speeds)) = common::signals::MOTORS_STATE.try_get() {
        //     rec.log(
        //         "sim/firmware/motors_cmd/m1",
        //         &Scalars::single(speeds[0] as f64),
        //     )?;
        //     rec.log(
        //         "sim/firmware/motors_cmd/m2",
        //         &Scalars::single(speeds[1] as f64),
        //     )?;
        //     rec.log(
        //         "sim/firmware/motors_cmd/m3",
        //         &Scalars::single(speeds[2] as f64),
        //     )?;
        //     rec.log(
        //         "sim/firmware/motors_cmd/m4",
        //         &Scalars::single(speeds[3] as f64),
        //     )?;
        // }

        if let Some([pid_x, pid_y, pid_z]) = common::signals::RATE_PID_TERMS.try_get() {
            rec.log(
                "sim/firmware/rate_pid/x/p",
                &Scalars::single(pid_x.p_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/x/i",
                &Scalars::single(pid_x.i_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/x/dr",
                &Scalars::single(pid_x.dr_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/x/dm",
                &Scalars::single(pid_x.dm_out as f64),
            )?;

            rec.log(
                "sim/firmware/rate_pid/y/p",
                &Scalars::single(pid_y.p_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/y/i",
                &Scalars::single(pid_y.i_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/y/dr",
                &Scalars::single(pid_y.dr_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/y/dm",
                &Scalars::single(pid_y.dm_out as f64),
            )?;

            rec.log(
                "sim/firmware/rate_pid/z/p",
                &Scalars::single(pid_z.p_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/z/i",
                &Scalars::single(pid_z.i_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/z/dr",
                &Scalars::single(pid_z.dr_out as f64),
            )?;
            rec.log(
                "sim/firmware/rate_pid/z/dm",
                &Scalars::single(pid_z.dm_out as f64),
            )?;
        }
        if let Some(quat) = common::signals::AHRS_ATTITUDE_Q.try_get() {
            let (x, y, z) = quat.euler_angles();
            rec.log("sim/firmware/ahrs_att/x", &Scalars::single(x as f64))?;
            rec.log("sim/firmware/ahrs_att/y", &Scalars::single(y as f64))?;
            rec.log("sim/firmware/ahrs_att/z", &Scalars::single(z as f64))?;
        }

        rec.log("sim/position/trail", &LineStrips3D::new([&pos_trail]))
            .unwrap();

        rec.log("sim/position/x", &Scalars::single(pos[0] as f64))
            .unwrap();
        rec.log("sim/position/y", &Scalars::single(pos[1] as f64))
            .unwrap();
        rec.log("sim/position/z", &Scalars::single(pos[2] as f64))
            .unwrap();

        rec.log("sim/rotation/x", &Scalars::single(rot[0] as f64))
            .unwrap();
        rec.log("sim/rotation/y", &Scalars::single(rot[1] as f64))
            .unwrap();
        rec.log("sim/rotation/z", &Scalars::single(rot[2] as f64))
            .unwrap();

        rec.log(
            "drone",
            &rerun::Transform3D::IDENTITY
                .with_translation(pos.data.0[0])
                .with_quaternion(rot.coords.data.0[0]),
        )?;
    }
}

pub fn log_drone_motors(
    rec: &RecordingStream,
    handle: SimHandle,
) -> Result<(), Box<dyn std::error::Error>> {
    // Make the arrows nice colors
    let make_color = |s: f32| -> Color {
        let g = ((3.5 - s) * 400.0) as u8;
        let r = ((s - 1.0) * 300.0) as u8;
        Color::from_rgb(r, g, 0)
    };

    // Combine the parameters and states of the motors
    let motors = handle
        .vehicle_params()
        .motors
        .iter()
        .cloned()
        .zip(handle.vehicle_state().motors)
        .collect::<Vec<_>>();

    let mut arrow_lengths = Vec::with_capacity(motors.len());
    let mut arrow_origins = Vec::with_capacity(motors.len());
    let mut arrow_colors = Vec::with_capacity(motors.len());

    for (idx, (params, state)) in motors.iter().enumerate() {
        rec.log(
            format!("drone/motor/force_m{}", idx),
            &Scalars::single(state.force),
        )?;
        rec.log(
            format!("drone/motor/command_m{}", idx),
            &Scalars::single(state.command),
        )?;

        // Shortening by 5 seems to work fine for small quads
        let direction = params.normal.normalize() * state.force / 5.0;

        arrow_lengths.push(Vec3D(direction.data.0[0]));
        arrow_origins.push(Vec3D(params.position.data.0[0]));
        arrow_colors.push(make_color(state.command));
    }

    rec.log(
        "drone/motors",
        &Arrows3D::from_vectors(arrow_lengths)
            .with_origins(arrow_origins)
            .with_colors(arrow_colors)
            .with_radii([0.01; 4]),
    )?;

    Ok(())
}
