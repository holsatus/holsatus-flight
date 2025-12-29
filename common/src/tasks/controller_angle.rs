use crate::{filters::angle_pid::Pid, get_ctrl_freq, get_or_warn, signals as sig};

mod params {
    use crate::tasks::param_storage::Table;

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct Parameters {
        pub roll: PidAxisParameters,
        pub pitch: PidAxisParameters,
        pub yaw: PidAxisParameters,
    }

    #[derive(Debug, Clone, mav_param::Tree)]
    pub struct PidAxisParameters {
        pub kp: f32,
        pub ki: f32,
        pub kd: f32,
        pub tau: f32,
    }

    crate::const_default!(
        Parameters => {
            roll: PidAxisParameters {
                kp: 15.,
                ki: 0.,
                kd: 0.0,
                tau: 0.0001,
            },
            pitch: PidAxisParameters {
                kp: 15.,
                ki: 0.,
                kd: 0.0,
                tau: 0.0001,
            },
            yaw: PidAxisParameters {
                kp: 25.,
                ki: 0.,
                kd: 0.03,
                tau: 0.001,
            }
        }
    );

    pub static TABLE: Table<Parameters> = Table::default("angl");
}

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "angle_loop";
    info!("{}: Task started", ID);

    let params = params::TABLE.read().await;

    // Task inputs
    // TODO - Use estimated attitude, not just IMU data
    let mut rcv_estimate = sig::ESKF_ESTIMATE.receiver();
    let mut rcv_attitude_sp = sig::TRUE_ATTITUDE_Q_SP.receiver();

    // Task outputs
    let mut snd_rate_sp = sig::ANGLE_TO_RATE_SP.sender();

    let dt = 1.0 / get_ctrl_freq!() as f32;

    let mut pid = [
        Pid::new(params.roll.kp, params.roll.ki, params.roll.kd, true, dt)
            .set_lp_filter(params.roll.tau),
        Pid::new(params.pitch.kp, params.pitch.ki, params.pitch.kd, true, dt)
            .set_lp_filter(params.pitch.tau),
        Pid::new(params.yaw.kp, params.yaw.ki, params.yaw.kd, true, dt)
            .set_lp_filter(params.yaw.tau),
    ];

    drop(params);

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
