use nalgebra::Vector3;

use crate::filters::pid_controller::Pid;
use crate::common::types::AttitudeReference;
use crate::messaging as msg;

#[embassy_executor::task]
pub async fn attitude_controller() -> ! {
    // Input messages
    let mut rcv_imu_data = msg::IMU_DATA.receiver().unwrap();
    let mut rcv_attitude_setpoint = msg::CMD_ATTITUDE_SETPOINT.receiver().unwrap();
    let mut rcv_attitude_euler = msg::ATTITUDE_EULER.receiver().unwrap();
    let mut rcv_en_integral = msg::CMD_EN_INTEGRAL.receiver().unwrap();

    // Output messages
    let snd_controller_output = msg::CONTROLLER_OUTPUT.sender();

    // Get static reference to config
    let pids = msg::CFG_ATTITUDE_PIDS.spin_get().await;

    // Period between IMU samples as seconds
    let imu_period = 1.0 / msg::CFG_LOOP_FREQUENCY.spin_get().await as f32;

    // Setup PID controllers for attitude (rate and angle) control
    let mut pid_roll_outer: _ = Pid::with_config(pids.roll_outer, imu_period);
    let mut pid_roll_inner: _ = Pid::with_config(pids.roll_inner, imu_period);
    let mut pid_pitch_outer: _ = Pid::with_config(pids.pitch_outer, imu_period);
    let mut pid_pitch_inner: _ = Pid::with_config(pids.pitch_inner, imu_period);
    let mut pid_yaw_outer: _ = Pid::with_config(pids.yaw_outer, imu_period);
    let mut pid_yaw_inner: _ = Pid::with_config(pids.yaw_inner, imu_period);

    // Start with integral controllers disabled
    pid_roll_outer.enable_integral(false);
    pid_roll_inner.enable_integral(false);
    pid_pitch_outer.enable_integral(false);
    pid_pitch_inner.enable_integral(false);
    pid_yaw_outer.enable_integral(false);
    pid_yaw_inner.enable_integral(false);

    let mut imu_data = rcv_imu_data.changed().await;

    '_infinite: loop {

        // Wait for new attitude estimate, and get new imu (gyro) data
        let attitude_euler = rcv_attitude_euler.changed().await;
        imu_data = rcv_imu_data.try_changed().unwrap_or(imu_data);

        // Enable or disable integral part of PID controllers
        if let Some(en_integral) = rcv_en_integral.try_changed() {
            pid_roll_outer.enable_reset_integral(en_integral);
            pid_roll_inner.enable_reset_integral(en_integral);
            pid_pitch_outer.enable_reset_integral(en_integral);
            pid_pitch_inner.enable_reset_integral(en_integral);
            pid_yaw_outer.enable_reset_integral(en_integral);
            pid_yaw_inner.enable_reset_integral(en_integral);
        }

        // Run PID controllers for attitude or rate control, depending on the selected mode
        let controller_output = match rcv_attitude_setpoint.try_get() {
            
            // Angle controller uses a cascaded PID control loop structure
            Some(AttitudeReference::Angle(angle_reference)) => {
                // Run outer part of cascaded control loop
                let outer_error = angle_reference - attitude_euler;
                let inner_reference = Vector3::new(
                    pid_roll_outer.update(outer_error.x),
                    pid_pitch_outer.update(outer_error.y),
                    pid_yaw_outer.update(outer_error.z),
                );

                // Run inner part of cascaded control loop
                let inner_error = inner_reference - imu_data.gyr;
                Vector3::new(
                    pid_roll_inner.update(inner_error.x),
                    pid_pitch_inner.update(inner_error.y),
                    pid_yaw_inner.update(inner_error.z),
                )
            }

            // Rate controller uses a single PID control loop
            Some(AttitudeReference::Rate(rate_reference)) => {
                let error = rate_reference - imu_data.gyr;
                Vector3::new(
                    pid_roll_inner.update(error.x),
                    pid_pitch_inner.update(error.y),
                    pid_yaw_inner.update(error.z),
                )
            }

            // No control output if no valid mode or rates are provided
            _ => Vector3::zeros(),
        };

        snd_controller_output.send(controller_output);
    }
}
