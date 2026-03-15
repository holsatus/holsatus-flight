use std::time::Duration;

use holsatus_sim::{Sim, SimHandle};
use rerun::{Arrows3D, Color, LineStrips3D, Points3D, RecordingStream, Scalars, Vec3D};

pub(crate) fn rerun_thread(
    rec: RecordingStream,
    handle: SimHandle,
) -> Result<(), Box<dyn std::error::Error>> {
    use std::collections::VecDeque;

    log::debug!("Starting rerun logging thread");

    rec.log_static("/", &rerun::ViewCoordinates::FRD())?;

    const LOG_FREQ: usize = 60;
    let mut log_ticker =
        crate::ticker::Ticker::every(Duration::from_secs_f32(1. / LOG_FREQ as f32));

    // Load 3D file for drone visualization
    let drone = rerun::Asset3D::from_file_path("fpv-drone-2.gltf")?;

    rec.log("drone", &drone)?;

    const TRAIL_LEN: usize = 1000;
    let mut pos_trail: VecDeque<Vec3D> = VecDeque::with_capacity(TRAIL_LEN);

    let mut mpc_intercept_pos = common::tasks::controller_mpc::MPC_INTERCEPT_POS.receiver();

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
                "sim/firmware/eskf/pos",
                &Scalars::new(estimate.pos.map(|x|x as f64).iter().cloned()),
            )?;

            rec.log(
                "sim/firmware/eskf/vel",
                &Scalars::new(estimate.vel.map(|x|x as f64).iter().cloned()),
            )?;

            let (roll, pitch, yaw) = estimate.att.euler_angles();
            rec.log(
                "sim/firmware/eskf/att",
                &Scalars::new([roll as f64, pitch as f64, yaw as f64]),
            )?;

            rec.log(
                "sim/firmware/eskf/gyr_bias",
                &Scalars::new(estimate.gyr_bias.map(|x|x as f64).iter().cloned()),
            )?;

            rec.log(
                "sim/firmware/eskf/acc_bias",
                &Scalars::new(estimate.acc_bias.map(|x|x as f64).iter().cloned()),
            )?;
        }

        if let Some(mpc_reference) = common::tasks::controller_mpc::MPC_REFERENCE.try_get() {
            let reference =
                mpc_reference.fixed_view::<3, { common::tasks::controller_mpc::HX }>(0, 0);
            let slices = reference
                .column_iter()
                .map(|col| col.clone_owned().data.0[0]);

            rec.log(
                "sim/firmware/mpc_reference_dot",
                &Points3D::new([reference.column(0).clone_owned().data.0[0]]).with_radii([0.05]),
            )?;

            rec.log("sim/firmware/mpc_reference", &LineStrips3D::new([slices]))?;
        }

        if let Some(mpc_pos_pred) = common::tasks::controller_mpc::MPC_POS_PRED.try_get() {
            let slices = mpc_pos_pred
                .column_iter()
                .map(|col| col.clone_owned().data.0[0]);

            rec.log(
                "sim/firmware/mpc_position_pred",
                &LineStrips3D::new([slices]),
            )?;
        }

        if let Some(intercept_pos) = mpc_intercept_pos.try_changed() {
            if let Some(intercept_pos) = intercept_pos {
                rec.log(
                    "sim/firmware/mpc_intercept_pos",
                    &Points3D::new([intercept_pos]).with_radii([1.0]),
                )?;
            } else {
                let empty: &[[f32; 3]] = &[]; 
                rec.log(
                    "sim/firmware/mpc_intercept_pos",
                    &Points3D::new(empty),
                )?;
            }
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
            rec.log("sim/firmware/angl_sp/x", &Scalars::single(roll as f64))?;
            rec.log("sim/firmware/angl_sp/y", &Scalars::single(pitch as f64))?;
            rec.log("sim/firmware/angl_sp/z", &Scalars::single(yaw as f64))?;
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

        let (roll, pitch, yaw) = rot.euler_angles();
        rec.log("sim/rotation/x", &Scalars::single(roll as f64))
            .unwrap();
        rec.log("sim/rotation/y", &Scalars::single(pitch as f64))
            .unwrap();
        rec.log("sim/rotation/z", &Scalars::single(yaw as f64))
            .unwrap();

        // Note: do not place this in the 'drone/' path since that will also apply
        // the drones rotation to this vector.
        if let Some(mpc_acc_target) = common::tasks::controller_mpc::MPC_TARGET_ACC.try_get() {
            let short_acc_target = mpc_acc_target.map(|x| x / 10.0);
            rec.log(
                "mpc_target_acc",
                &Arrows3D::from_vectors([&short_acc_target])
                    .with_radii([0.02])
                    .with_origins([pos.data.0[0]]),
            )?;
        }

        rec.log(
            "drone",
            &rerun::Transform3D::from_translation(pos.data.0[0])
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
