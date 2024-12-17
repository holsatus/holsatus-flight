use core::f32::consts::PI;

use embassy_sync::watch::{DynReceiver, DynSender};
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
    let mut rcv_attitude = sig::AHRS_ATTITUDE_Q.receiver().unwrap();
    let mut rcv_angle_sp = sig::TRUE_ANGLE_SP.receiver().unwrap();

    // Task outputs
    let snd_rate_sp = sig::ANGLE_TO_RATE_SP.sender();

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




struct AngleLoopTask {
    attitude: DynReceiver<'static, UnitQuaternion<f32>>,
    angle_sp: DynReceiver<'static, [f32; 3]>,
    rate_sp: DynSender<'static, [f32; 3]>,
    pid: [Pid<f32>; 3],
    slew: [SlewRate<f32>; 3],
}

impl AngleLoopTask {
    pub fn new(
        attitude: DynReceiver<'static, UnitQuaternion<f32>>,
        angle_sp: DynReceiver<'static, [f32; 3]>,
        rate_sp: DynSender<'static, [f32; 3]>,
    ) -> Self {
        let ts = crate::ANGLE_LOOP_DIV as f32 / crate::MAIN_LOOP_FREQ as f32;

        let pid = [
            Pid::new(10., 0.0, 0.01, true, ts).set_lp_filter(0.002).set_wrapping(-PI, PI),
            Pid::new(10., 0.0, 0.01, true, ts).set_lp_filter(0.002).set_wrapping(-PI, PI),
            Pid::new(2., 0.0, 0.00, true, ts).set_lp_filter(0.002).set_wrapping(-PI, PI),
        ];

        let slew = [
            SlewRate::new(PI * 8.0, ts),
            SlewRate::new(PI * 8.0, ts),
            SlewRate::new(5., ts),
        ];

        Self {
            attitude,
            angle_sp,
            rate_sp,
            pid,
            slew,
        }
    }

    pub async fn run(&mut self) -> () {
        let attitude = self.attitude.changed().await;
        let angle_sp = self.angle_sp.get().await.map(|e|e / 12.);

        let slew_sp = [
            self.slew[0].update(angle_sp[0]),
            self.slew[1].update(angle_sp[1]),
            self.slew[2].update(angle_sp[2]),
        ];

        let angle_sp = UnitQuaternion::from_euler_angles(slew_sp[0], slew_sp[1], slew_sp[2]);

        let error = attitude.rotation_to(&angle_sp);
        let euler = error.euler_angles();

        let rate_sp = [
            self.pid[0].update(euler.0),
            self.pid[1].update(euler.1),
            self.pid[2].update(euler.2),
        ];

        self.rate_sp.send(rate_sp);
    }
}

#[embassy_executor::task]
pub async fn main2() -> ! {
    let mut angle_loop_task = AngleLoopTask::new(
        sig::AHRS_ATTITUDE_Q.dyn_receiver().unwrap(),
        sig::TRUE_ANGLE_SP.dyn_receiver().unwrap(),
        sig::ANGLE_TO_RATE_SP.dyn_sender(),
    );

    loop {
        angle_loop_task.run().await;
    }
}