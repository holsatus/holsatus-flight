use core::f32::consts::PI;

use crate::{
    filters::angle_pid::Pid, get_ctrl_freq, get_or_warn, signals as sig
};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "angle_loop";
    info!("{}: Task started", ID);

    // Task inputs
    // TODO - Use estimated attitude, not just IMU data
    let mut rcv_attitude = sig::AHRS_ATTITUDE_Q.receiver();
    let mut rcv_attitude_sp = sig::TRUE_ATTITUDE_Q_SP.receiver();

    // Task outputs
    let mut snd_rate_sp = sig::ANGLE_TO_RATE_SP.sender();

    let ts = 1.0 / get_ctrl_freq!() as f32;

    let mut pid = [
        Pid::new(25., 0.0, 0.04, true, ts)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
        Pid::new(25., 0.0, 0.04, true, ts)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
        Pid::new(10., 0.0, 0.01, true, ts)
            .set_lp_filter(0.001)
            .set_wrapping(-PI, PI),
    ];

    get_or_warn!(rcv_attitude).await;
    get_or_warn!(rcv_attitude_sp).await;

    info!("{}: Entering main loop", ID);
    loop {
        let q_attitude = rcv_attitude.changed().await;
        let q_setpoint = rcv_attitude_sp.get().await;

        // Using the "quaternion error" rather than the euler angle error gives some
        // much nicer behavior where euler angles would normally experiece gimbal lock.
        let q_error = q_setpoint * q_attitude.conjugate();

        let rate_sp = [
            pid[0].update(q_error.i),
            pid[1].update(q_error.j),
            pid[2].update(q_error.k),
        ];

        snd_rate_sp.send(rate_sp);
    }
}
