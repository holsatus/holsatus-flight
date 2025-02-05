use core::f32::consts::PI;

use nalgebra::UnitQuaternion;

use crate::{
    filters::{angle_pid::Pid, SlewRate}, get_or_warn, signals as sig
};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "angle_loop";
    info!("{}: Task started", ID);

    // Task inputs
    // TODO - Use estimated attitude, not just IMU data
    let mut rcv_attitude = sig::AHRS_ATTITUDE_Q.receiver();
    let mut rcv_angle_sp = sig::TRUE_ANGLE_SP.receiver();

    // Task outputs
    let mut snd_rate_sp = sig::ANGLE_TO_RATE_SP.sender();

    let ts = crate::ANGLE_LOOP_DIV as f32 / crate::MAIN_LOOP_FREQ as f32;

    let mut pid = [
        Pid::new(10., 0.0, 0.01, true, ts)
            .set_lp_filter(0.002)
            .set_wrapping(-PI, PI),
        Pid::new(10., 0.0, 0.01, true, ts)
            .set_lp_filter(0.002)
            .set_wrapping(-PI, PI),
        Pid::new(2., 0.0, 0.00, true, ts)
            .set_lp_filter(0.002)
            .set_wrapping(-PI, PI),
    ];

    let mut slew = [
        SlewRate::new(PI * 8.0, ts),
        SlewRate::new(PI * 8.0, ts),
        SlewRate::new(5., ts),
    ];

    get_or_warn!(rcv_attitude).await;
    get_or_warn!(rcv_angle_sp).await;

    info!("{}: Entering main loop", ID);
    loop {
        let attitude = rcv_attitude.changed().await;
        let angle_sp = rcv_angle_sp.get().await.map(|e|e / 12.);

        let slew_sp = [
            slew[0].update(angle_sp[0]),
            slew[1].update(angle_sp[1]),
            slew[2].update(angle_sp[2]),
        ];

        let angle_sp = UnitQuaternion::from_euler_angles(slew_sp[0], slew_sp[1], slew_sp[2]);

        let error = attitude.rotation_to(&angle_sp);
        let euler = error.euler_angles();

        let rate_sp = [
            pid[0].update(euler.0),
            pid[1].update(euler.1),
            pid[2].update(euler.2),
        ];

        snd_rate_sp.send(rate_sp);
    }
}
