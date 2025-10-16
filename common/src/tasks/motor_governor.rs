use embassy_futures::select::{
    select,
    Either::{First, Second},
};
use embassy_time::{with_timeout, Duration, Timer};
#[cfg(feature = "mavlink")]
use mavio::default_dialect::enums::MavModeFlag;

use crate::{
    filters::{motor_lin::MotorLin, Linear},
    tasks::motor_governor::params::Reverse,
};
use crate::{
    hw_abstraction::OutputGroup,
    types::actuators::{DisarmReason, MotorsState},
};
use crate::{signals as s, MOTOR_TIMEOUT_MS};

pub mod params {
    use crate::tasks::param_storage::Table;

    #[derive(mav_param::Tree, Clone, Debug)]
    pub struct Params {
        pub rev: Reverse,
        pub timeout_ms: u16,
        pub lin: Linearizer,
        pub out_min: f32,
        pub out_max: f32,
    }

    crate::const_default!(
        Params => {
            rev: Reverse::empty(),
            timeout_ms: 100,
            lin: Linearizer::const_default(),
            out_min: crate::DSHOT_MIN as f32,
            out_max: crate::DSHOT_MAX as f32,
        }
    );

    #[derive(mav_param::Node, Clone, Debug)]
    pub struct Reverse(u16);

    bitflags::bitflags! {
        impl Reverse: u16 {
            const MOTOR_0 = 1 << 0;
            const MOTOR_1 = 1 << 1;
            const MOTOR_2 = 1 << 2;
            const MOTOR_3 = 1 << 3;
            const MOTOR_4 = 1 << 4;
            const MOTOR_5 = 1 << 5;
            const MOTOR_6 = 1 << 6;
            const MOTOR_7 = 1 << 7;
        }
    }

    crate::const_default!(
        Reverse => Reverse::MOTOR_1.union(Reverse::MOTOR_2)
    );

    #[derive(mav_param::Tree, Clone, Debug)]
    pub struct Linearizer {
        pub a: f32,
        pub b: f32,
    }

    crate::const_default!(
        Linearizer => {
            a: 10.452,
            b: 0.55,
        }
    );

    pub static TABLE: Table<Params> = Table::new("mtr", Params::const_default());
}

pub async fn main(mut motors: impl OutputGroup) -> ! {
    const ID: &str = "motor_governor";
    info!("{}: Task started", ID);

    // Input signals
    let mut rcv_comand_arm = crate::tasks::commander::COMMAD_ARM_VEHICLE.receiver();
    let mut rcv_motors_mixed = crate::tasks::controller_rate::RATE_MOTORS_MIXED.receiver();

    // Output signals
    let mut snd_motors_state = s::MOTORS_STATE.sender();

    let params = params::TABLE.read_initialized().await;
    debug!("[{}] Received initial parameters", ID);

    let mut dirs;

    let timeout_dur = Duration::from_millis(MOTOR_TIMEOUT_MS as u64);

    // Linearize the non-linear command->thrust curve of the motor+propeller
    let thrust_lin = MotorLin::new(params.lin.a, params.lin.b, 0.05, 1.0);

    let (min, max) = (params.out_min, params.out_max);
    drop(params);

    // Map the normalized control signal to valid range of driver
    let dshot_map = Linear::new(0., 1., min, max);

    // Function which maps each incoming channel into a command value
    let dshot_map_fn = |thrust: f32| {
        let command = thrust_lin.force_to_command(thrust);
        dshot_map.map(command).clamp(min, max as f32) as u16
    };

    // Set initial state
    snd_motors_state.send(MotorsState::Disarmed(DisarmReason::Uninitialized));
    'disarmed: loop {
        #[cfg(feature = "mavlink")]
        crate::mavlink2::mav_mode::set(MavModeFlag::SAFETY_ARMED, false);

        // Await arming command
        rcv_comand_arm.changed_and(|&arm| arm == true).await;
        snd_motors_state.send(MotorsState::Arming);

        // Ensure we have latest configuration
        let params = params::TABLE.read_initialized().await;

        dirs = [
            params.rev.contains(Reverse::MOTOR_0),
            params.rev.contains(Reverse::MOTOR_1),
            params.rev.contains(Reverse::MOTOR_2),
            params.rev.contains(Reverse::MOTOR_3),
        ];

        drop(params);

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

        // TODO - Ask the commander whether we are still clear to arm?

        #[cfg(feature = "mavlink")]
        crate::mavlink2::mav_mode::set(MavModeFlag::SAFETY_ARMED, true);
        snd_motors_state.send(MotorsState::ArmedIdle);

        info!("{}: Entering main loop", ID);
        'armed: loop {
            match select(
                rcv_comand_arm.changed_and(|&arm| arm == false),
                with_timeout(timeout_dur, rcv_motors_mixed.changed()),
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
                    let speeds_u16 = speeds.map(dshot_map_fn);
                    motors.set_motor_speeds(speeds_u16).await;
                    snd_motors_state.send(MotorsState::Armed(speeds_u16));
                    continue 'armed;
                }
            }
        }
    }
}
