use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_time::{with_timeout, Duration, Timer};

use crate::{filters::{motor_lin::MotorLin, Linear}, tasks::imu_reader::TIME_SIG, DSHOT_MAX, DSHOT_MIN};
use crate::{
    hw_abstraction::FourMotors,
    types::actuators::{DisarmReason, MotorsState},
};
use crate::{signals as s, MOTOR_TIMEOUT_MS};

pub async fn main(mut motors: impl FourMotors) -> ! {
    const ID: &str = "motor_governor";
    info!("{}: Task started", ID);

    // Input signals
    let mut rcv_arm_blocker = unwrap!(s::ARMING_BLOCKER.receiver());
    let mut rcv_arm_command = unwrap!(s::CMD_ARM_MOTORS.receiver());
    let mut rcv_motor_speeds = unwrap!(s::CTRL_MOTORS.receiver());
    let mut rcv_cfg_motor_dirs = unwrap!(s::CFG_MOTOR_DIRS.receiver());

    // Output signals
    let snd_motors_state = s::MOTORS_STATE.sender();

    let mut dirs = crate::get_or_warn!(rcv_cfg_motor_dirs).await;
    
    let timeout_dur = Duration::from_millis(MOTOR_TIMEOUT_MS as u64);

    // Linearize the non-linear command->thrust curve of the motor+propeller
    let thrust_lin = MotorLin::new(10.452, 0.55, 0.05, 1.0);

    // Map the normalized control signal to valid DShot values
    let dshot_map = Linear::new(0., 1., DSHOT_MIN as f32, DSHOT_MAX as f32);

    // Function which maps each incoming channel into a command value
    let dshot_map_fn = |x: f32| {
        let x = thrust_lin.command_linearized(x);
        dshot_map.map(x.clamp(0.01, 1.)) as u16
    };

    // Set initial state
    snd_motors_state.send(MotorsState::Disarmed(DisarmReason::Uninitialized));
    'disarmed: loop {

        #[cfg(feature = "mavlink")]
        crate::mavlink::mav_set_safety_armed(false);

        // Await arming command
        let (arm, force) = rcv_arm_command.changed().await;
        if !arm { continue 'disarmed }
        
        snd_motors_state.send(MotorsState::Arming);

        // Ensure we have latest configuration
        dirs = rcv_cfg_motor_dirs.try_changed().unwrap_or(dirs);

        // Send minimum throttle for a few seconds to start arming
        info!("{}: Arming motors", ID);
        Timer::after_millis(1000).await;
        for _ in 0..50 {
            motors.set_motor_speeds_min().await;
            Timer::after_millis(50).await;
        }

        // Set motor directions for the four motors
        info!("{}: Setting directions {:?}", ID, dirs);
        for _ in 0..20 {
            motors.set_reverse_dir(dirs).await;
            Timer::after_millis(50).await;
        }

        // If after arming, the arming blocker active, and the output override
        // is not set, disarm the motors
        if !(force || rcv_arm_blocker.get().await.is_empty()) {
            warn!("{}: Disarming motors -> arming blocker", ID);
            snd_motors_state.send(MotorsState::Disarmed(DisarmReason::ArmingBlocker));
            continue 'disarmed;
        }
        
        #[cfg(feature = "mavlink")]
        crate::mavlink::mav_set_safety_armed(true);
        snd_motors_state.send(MotorsState::ArmedIdle);
        let mut total_duration = Duration::from_ticks(0);
        let mut duration_count = 0;

        info!("{}: Entering main loop", ID);
        'armed: loop {
            match select(
                rcv_arm_command.changed_and(|&(x, _)|x == false),
                with_timeout(timeout_dur, rcv_motor_speeds.changed()),
            )
            .await
            {
                First(_) => {
                    motors.set_motor_speeds_min().await;
                    info!("{}: Disarming motors as commanded", ID);
                    snd_motors_state.send(MotorsState::Disarmed(DisarmReason::UserCommand));
                    continue 'disarmed;
                }
                Second(Err(_)) => {
                    motors.set_motor_speeds_min().await;
                    warn!("{}: Disarming motors due to timeout", ID);
                    snd_motors_state.send(MotorsState::Disarmed(DisarmReason::Timeout));
                    continue 'disarmed;
                }
                Second(Ok([0., 0., 0., 0.])) => {
                    motors.set_motor_speeds_min().await;
                    snd_motors_state.send(MotorsState::ArmedIdle);
                    continue 'armed;
                }
                Second(Ok(speeds)) => {
                    let _speeds_u16 = speeds.map(dshot_map_fn);

                    if let Some(time) = TIME_SIG.try_take() {
                        let duration = time.elapsed();
                        total_duration += duration;
                        duration_count += 1;
                    }

                    if !(duration_count < 1000) {
                        let avg_duration = total_duration / 1000;
                        info!("{}: Average loop duration: {:?}", ID, avg_duration);
                        total_duration = Duration::from_ticks(0);
                        duration_count = 0;
                    }

                    // motors.set_motor_speeds(speeds_u16).await;
                    // snd_motors_state.send(MotorsState::Armed(speeds_u16));
                    continue 'armed;
                }
            }
        }
    }
}
