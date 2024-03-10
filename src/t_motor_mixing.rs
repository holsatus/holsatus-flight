use embassy_futures::select::select;

use crate::{functions::map, health::LoopHealth, messaging as msg};

#[embassy_executor::task]
pub async fn motor_mixing() -> ! {

    // Input messages
    let mut rcv_controller_output = msg::CONTROLLER_OUTPUT.receiver().unwrap();
    let mut rcv_throttle_setpoint = msg::CMD_THROTTLE_SETPOINT.receiver().unwrap();
    let mut rcv_cfg_mixing_matrix = msg::CFG_MIXING_MATRIX.receiver().unwrap();

    // Output messages
    let snd_motor_speeds = msg::MOTOR_SPEEDS.sender();
    let snd_loop_health = msg::LOOP_HEALTH.sender();

    let mut mixing_matrix = rcv_cfg_mixing_matrix.changed().await;

    // Initialize loop health monitor
    let mut loop_health = LoopHealth::new(msg::CFG_LOOP_FREQUENCY.spin_get().await);

    '_infinite: loop {

        match select(
            rcv_controller_output.changed(),
            rcv_cfg_mixing_matrix.changed(),
        ).await {
            embassy_futures::select::Either::First(controller_output) => {
                let throttle_target = rcv_throttle_setpoint.try_get().unwrap_or(0.0);

                // Map raw throttle command [0..1] to usable range [200..1500]
                let throttle_mapped = map(throttle_target, 0.0, 1.0, 200., 1500.);

                // Mix thrust and controller output into motor speeds
                let motors_mixed = mixing_matrix.mixing_fn(throttle_mapped, controller_output)
                    .map(|x| (x as u16).clamp(70, 1500));

                // Publish motor speeds to motor governor task
                snd_motor_speeds.send(motors_mixed.into());

                // Evaluate and send loop health
                if loop_health.evaluate() {
                    snd_loop_health.send(loop_health.get_health());
                }
            }
            embassy_futures::select::Either::Second(new_mixing_matrix) => {
                // Update the mixing matrix
                mixing_matrix = new_mixing_matrix;
            }
        }

        
    }
}
