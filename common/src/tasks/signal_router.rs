use futures::{select_biased, FutureExt};
use nalgebra::UnitQuaternion;

use crate::{
    get_or_warn, signals as s, tasks::commander::CMD_CONTROL_MODE, types::control::ControlMode,
};

pub mod param {
    use crate::tasks::{
        param_storage::Table,
        rc_binder::rates::{Actual, Linear, Rates},
    };

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        pub rate: [Rates; 3],
        pub angl: [Rates; 3],
        pub throt: Rates,
    }

    crate::const_default!(
        Params => {
            rate: [const { Rates::Actual(Actual::const_default()) }; 3],
            angl: [const { Rates::Linear(Linear::const_default()) }; 3],
            throt: Rates::Linear(Linear::const_default()),
        }
    );

    pub static TABLE: Table<Params> = Table::new("fn", Params::const_default());
}

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "signal_router";
    info!("{}: Task started", ID);

    let params = param::TABLE.read_initialized().await.clone();

    // Get inputs
    let mut rcv_control_mode = CMD_CONTROL_MODE.receiver();
    let mut rcv_rc_controls = s::RC_ANALOG_UNIT.receiver();
    let mut rcv_angle_to_rate_sp = s::ANGLE_TO_RATE_SP.receiver();
    let mut rcv_vel_to_angle_sp = s::VEL_TO_ANGLE_SP.receiver();
    let mut rcv_mpc_angle_sp = crate::tasks::controller_mpc::MPC_TARGET_ATT.receiver();
    let mut rcv_mpc_force_sp = crate::tasks::controller_mpc::MPC_TARGET_FRC.receiver();

    // Get outputs
    let mut snd_rate_sp = s::TRUE_RATE_SP.sender();
    let mut snd_angle_sp = s::TRUE_ATTITUDE_Q_SP.sender();
    let mut snd_z_thrust_sp = s::TRUE_Z_THRUST_SP.sender();
    let mut snd_velocity_sp = s::TRUE_VELOCITY_SP.sender();

    // Wait for the control mode to be defiend before routing starts
    let mut control_mode = get_or_warn!(rcv_control_mode).await;
    info!(
        "{}: Setting initial control mode to: {:?}",
        ID, control_mode
    );

    loop {
        // Listen to all signals in order of priority, and handle them asyncronously.
        select_biased! {

            // Update the control mode
            new_control_mode = rcv_control_mode.changed().fuse() => {
                info!("{}: Changed mode to: {:?}", ID, new_control_mode);
                control_mode = new_control_mode;
            },

            // Route the RC controls to the appropriate setpoints
            rc_controls = rcv_rc_controls.changed().fuse() => {
                // We use a CS here to ensure that the controller signal is propagated
                // without interruption, and that attitude and throttle are updated atomically.
                critical_section::with(|_|{
                    match control_mode {
                        ControlMode::Rate => {

                            let mut rate_sp = rc_controls.roll_pitch_yaw();
                            for i in 0..3 {
                                rate_sp[i] = params.rate[i].apply(rate_sp[i]);
                            }

                            let throttle_sp = params.throt.apply(rc_controls.throttle());

                            snd_rate_sp.send(rate_sp);
                            snd_z_thrust_sp.send(throttle_sp);
                        },
                        ControlMode::Angle => {

                            let mut angle_sp = rc_controls.roll_pitch_yaw();
                            for i in 0..3 {
                                angle_sp[i] = params.angl[i].apply(angle_sp[i]);
                            }

                            let throttle_sp = params.throt.apply(rc_controls.throttle());
                            let angle_sp = UnitQuaternion::from_euler_angles(angle_sp[0], angle_sp[1], angle_sp[2]);

                            snd_angle_sp.send(angle_sp);
                            snd_z_thrust_sp.send(throttle_sp);
                        },
                        ControlMode::Velocity => {
                            // The throttle is controlled by a separate controller
                            snd_velocity_sp.send(rc_controls.roll_pitch_yaw());
                        },
                        ControlMode::Autonomous => {
                            /* Do nothing with RC input in auto mode? */
                            /* Maybe inputs can be used to guide system */
                        }
                    }
                })
            },

            // Route the angle setpoints to the rate controller
            rate_sp = rcv_angle_to_rate_sp.changed().fuse() => {
                match control_mode {
                    ControlMode::Angle | ControlMode::Velocity | ControlMode::Autonomous => {
                        snd_rate_sp.send(rate_sp);
                    }
                    _ => ()
                }
            },

            // Route the angle setpoints to the rate controller
            angle_sp = rcv_vel_to_angle_sp.changed().fuse() => {
                if control_mode == ControlMode::Velocity {
                    snd_angle_sp.send(angle_sp);
                }
            },

            // Route MPC-generated attitude into angle controller
            mpc_angle_sp = rcv_mpc_angle_sp.changed().fuse() => {
                if control_mode == ControlMode::Autonomous {
                    snd_angle_sp.send(mpc_angle_sp);
                }
            }

            // Route MPC-generated total thrust force
            mpc_force_sp = rcv_mpc_force_sp.changed().fuse() => {
                if control_mode == ControlMode::Autonomous {
                    snd_z_thrust_sp.send(mpc_force_sp);
                }
            }
        };
    }
}
