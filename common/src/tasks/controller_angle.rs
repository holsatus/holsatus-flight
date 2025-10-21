use core::f32::consts::PI;

use nalgebra::{Quaternion, SVector, Unit};

use crate::{filters::angle_pid::Pid, get_ctrl_freq, get_or_warn, signals as sig};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "angle_loop";
    info!("{}: Task started", ID);

    // Task inputs
    // TODO - Use estimated attitude, not just IMU data
    let mut rcv_estimate = sig::ESKF_ESTIMATE.receiver();
    let mut rcv_attitude = sig::AHRS_ATTITUDE_Q.receiver();
    let mut rcv_attitude_sp = sig::TRUE_ATTITUDE_Q_SP.receiver();

    // Task outputs
    let mut snd_rate_sp = sig::ANGLE_TO_RATE_SP.sender();

    let dt = 1.0 / get_ctrl_freq!() as f32;

    let mut pid = [
        Pid::new(15., 0.0, 0.05, true, dt)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
        Pid::new(15., 0.0, 0.05, true, dt)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
        Pid::new(15., 0.0, 0.02, true, dt)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
    ];

    get_or_warn!(rcv_attitude).await;
    get_or_warn!(rcv_attitude_sp).await;

    info!("{}: Entering main loop", ID);
    loop {
        //  Run at quickly as we get an attitude estimate
        let q_attitude = rcv_estimate.changed().await.att;
        let q_setpoint = rcv_attitude_sp.get().await;

        // Using the "quaternion error" rather than the euler angle error gives some
        // much nicer behavior where euler angles would normally experiece gimbal lock.
        let q_error = q_attitude.inverse() * q_setpoint;
        let axis_error = q_error.scaled_axis();

        let rate_sp = [
            pid[0].update(axis_error.x),
            pid[1].update(axis_error.y),
            pid[2].update(axis_error.z),
        ];

        snd_rate_sp.send(rate_sp);
    }
}
