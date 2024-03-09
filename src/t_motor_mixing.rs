use crate::{functions::map, health::LoopHealth, messaging as msg};

#[embassy_executor::task]
pub async fn motor_mixing() -> ! {

    // Input messages
    let mut rcv_controller_output = msg::CONTROLLER_OUTPUT.receiver().unwrap();
    let mut rcv_throttle_setpoint = msg::THROTTLE_SETPOINT.receiver().unwrap();

    // Output messages
    let snd_motor_speeds = msg::MOTOR_SPEEDS.sender();
    let snd_loop_health = msg::LOOP_HEALTH.sender();

    // Get static reference to config
    let config = msg::STATIC_CONFIG_REF.receiver().unwrap().changed().await;

    // Initialize loop health monitor
    let mut loop_health = LoopHealth::new(config.imu_freq);

    '_infinite: loop {

        // Wait for the next controller output
        let controller_output = rcv_controller_output.changed().await;
        let throttle_target = rcv_throttle_setpoint.try_get().unwrap_or(0.0);

        // Map raw throttle command [0..1] to usable range [200..1500]
        let throttle_mapped = map(throttle_target, 0.0, 1.0, 200., 1500.);

        // Mix thrust and controller output into motor speeds
        let motors_mixed =
            config.mixing_matrix.mixing_fn(throttle_mapped, controller_output)
            .map(|x| (x as u16).clamp(70, 1500));

        // Publish motor speeds to motor governor task
        snd_motor_speeds.send(motors_mixed.into());

        // Evaluate and send loop health
        if loop_health.evaluate() {
            snd_loop_health.send(loop_health.get_health());
        }
    }
}
