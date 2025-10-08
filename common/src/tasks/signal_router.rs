use futures::{select_biased, FutureExt};

use crate::{get_or_warn, signals as s, types::control::ControlMode};

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "signal_router";
    info!("{}: Task started", ID);

    // Get inputs
    let mut rcv_control_mode = s::CONTROL_MODE.receiver();
    let mut rcv_rc_controls = s::RC_ANALOG_RATE.receiver();
    let mut rcv_angle_to_rate_sp = s::ANGLE_TO_RATE_SP.receiver();
    let mut rcv_vel_to_angle_sp = s::VEL_TO_ANGLE_SP.receiver();

    // Get outputs
    let mut snd_rate_sp = s::TRUE_RATE_SP.sender();
    let mut snd_angle_sp = s::TRUE_ANGLE_SP.sender();
    let mut snd_throttle_sp = s::TRUE_THROTTLE_SP.sender();
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
                            snd_rate_sp.send(rc_controls.roll_pitch_yaw());
                            snd_throttle_sp.send(rc_controls.throttle());
                        },
                        ControlMode::Angle => {
                            snd_angle_sp.send(rc_controls.roll_pitch_yaw());
                            snd_throttle_sp.send(rc_controls.throttle());
                        },
                        ControlMode::Velocity => {
                            // The throttle is controlled by a separate controller
                            snd_velocity_sp.send(rc_controls.roll_pitch_yaw());
                        },
                    }
                })
            },

            // Route the angle setpoints to the rate controller
            rate_sp = rcv_angle_to_rate_sp.changed().fuse() => {
                if control_mode == ControlMode::Angle {
                    snd_rate_sp.send(rate_sp);
                }
            },

            // Route the angle setpoints to the rate controller
            angle_sp = rcv_vel_to_angle_sp.changed().fuse() => {
                if control_mode == ControlMode::Velocity {
                    snd_angle_sp.send(angle_sp);
                }
            },
        };
    }
}
