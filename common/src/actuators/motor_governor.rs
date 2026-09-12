use embassy_futures::select::{Either, Either3, select, select3};
use embassy_time::{Duration, Ticker, Timer, with_timeout};

use crate::{
    filters::{Linear, Lowpass, motor_lin::MotorLin},
    multicopter::attitude_control::MOTORS_MIXED,
    signals::MOTORS_STATE,
    sync::{channel::Channel, watch::Sender},
    tasks::commander::COMMAD_ARM_VEHICLE,
    types::actuators::OutputsRaw,
};

use params::Reverse;

use crate::sync::watch::Receiver;
use crate::{
    abstraction::motor::MotorGroup,
    types::actuators::{DisarmReason, MotorsState},
};

pub mod params {
    use crate::params::ParamTable;

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
            rev: Reverse::const_default(),
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
            const MOTOR_1 = 1 << 0;
            const MOTOR_2 = 1 << 1;
            const MOTOR_3 = 1 << 2;
            const MOTOR_4 = 1 << 3;
        }
    }

    crate::const_default!(
        Reverse => Reverse::empty()
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

    pub static TABLE: ParamTable<Params> = ParamTable::default("mtr");
}

struct MotorGovernor<'a, O> {
    motors: O,
    reverse: [bool; 4],
    timeout: Duration,
    unscale: Linear<f32>,
    motor_map: MotorLin<f32>,
    lowpass: [Lowpass<f32>; 4],
    recv_arm: Receiver<'a, bool>,
    recv_motors: Receiver<'a, [f32; 4]>,
    send_state: Sender<'a, MotorsState>,
}

enum State {
    Disarmed,
    Armed,
}

pub enum Message {
    ReloadParams,
}

static CHANNEL: Channel<Message, 1> = Channel::new();

pub async fn main(output: impl MotorGroup) -> ! {
    let mut state = MotorGovernor::new(
        output,
        &*params::TABLE.read().await,
        COMMAD_ARM_VEHICLE.receiver(),
        MOTORS_MIXED.receiver(),
        MOTORS_STATE.sender(),
    );

    state.run().await
}

impl<'a, M: MotorGroup> MotorGovernor<'a, M> {
    fn new(
        motors: M,
        params: &params::Params,
        recv_arm: Receiver<'a, bool>,
        recv_motors: Receiver<'a, [f32; 4]>,
        send_state: Sender<'a, MotorsState>,
    ) -> Self {
        let timeout = Duration::from_millis(params.timeout_ms as u64);
        let motor_map = MotorLin::new(params.lin.a, params.lin.b, 0.05, 1.0);
        let unscale = Linear::new(0., 1., params.out_min, params.out_max);

        let reverse = [
            params.rev.contains(Reverse::MOTOR_1),
            params.rev.contains(Reverse::MOTOR_2),
            params.rev.contains(Reverse::MOTOR_3),
            params.rev.contains(Reverse::MOTOR_4),
        ];

        Self {
            motors,
            reverse,
            unscale,
            timeout,
            motor_map,
            lowpass: [Lowpass::new(0.001, 0.001); 4],
            recv_arm,
            recv_motors,
            send_state,
        }
    }

    fn reconfigure(&mut self, params: &params::Params) {
        self.timeout = Duration::from_millis(params.timeout_ms as u64);
        self.motor_map = MotorLin::new(params.lin.a, params.lin.b, 0.05, 1.0);
        self.unscale = Linear::new(0., 1., params.out_min, params.out_max);

        self.reverse = [
            params.rev.contains(Reverse::MOTOR_1),
            params.rev.contains(Reverse::MOTOR_2),
            params.rev.contains(Reverse::MOTOR_3),
            params.rev.contains(Reverse::MOTOR_4),
        ];
    }

    async fn run(&mut self) -> ! {
        let mut state = State::Disarmed;
        self.startup_sequence().await;
        loop {
            state = match state {
                State::Disarmed => self.run_disarmed().await,
                State::Armed => self.run_armed().await,
            };
        }
    }

    async fn startup_sequence(&mut self) {
        self.send_state
            .send(MotorsState::Disarmed(DisarmReason::Uninitialized));

        Timer::after_millis(2000).await;

        let mut ms_ticker = Ticker::every(Duration::from_millis(1));

        for _ in 0..1000 {
            self.motors.set_motor_speeds_min().await;
            ms_ticker.next().await;
        }

        for _ in 0..1000 {
            self.motors.set_reverse_dir(self.reverse).await;
            ms_ticker.next().await;
        }
    }

    async fn run_disarmed(&mut self) -> State {
        let mut ticker = Ticker::every(Duration::from_hz(100));
        loop {
            match select3(
                self.recv_arm.changed_and(|&arm| arm == true),
                ticker.next(),
                CHANNEL.receive(),
            )
            .await
            {
                Either3::First(_) => {
                    info!("[motor_governor]: Arming motors as commanded");
                    return State::Armed;
                }
                Either3::Second(()) => {
                    self.motors.set_motor_speeds_min().await;
                }
                Either3::Third(message) => match message {
                    Message::ReloadParams => {
                        let params = params::TABLE.read().await;
                        self.reconfigure(&params)
                    }
                },
            }
        }
    }

    async fn run_armed(&mut self) -> State {
        loop {
            match select(
                self.recv_arm.changed_and(|&arm| arm == false),
                with_timeout(self.timeout, self.recv_motors.changed()),
            )
            .await
            {
                Either::First(_) => {
                    info!("[motor_governor]: Disarming motors as commanded");
                    self.motors.set_motor_speeds_min().await;

                    let state = MotorsState::Disarmed(DisarmReason::UserCommand);
                    self.send_state.send(state);
                    return State::Disarmed;
                }
                Either::Second(Err(_)) => {
                    warn!("[motor_governor]: Disarming motors due to timeout");
                    self.motors.set_motor_speeds_min().await;

                    let state = MotorsState::Disarmed(DisarmReason::Timeout);
                    self.send_state.send(state);
                    return State::Disarmed;
                }
                Either::Second(Ok(mut speeds)) => {
                    speeds = speeds.map(|x1| {
                        let x2 = self.motor_map.force_to_command(x1);
                        let x3 = self.unscale.map(x2);
                        // info!("x1: {x1}, x2: {x2}, x3: {x3}");
                        x3
                    });

                    let speeds_u16 =
                        core::array::from_fn(|idx| self.lowpass[idx].update(speeds[idx]) as u16);

                    self.motors.set_motor_speeds(speeds_u16).await;

                    let state = MotorsState::Armed(OutputsRaw(speeds_u16));
                    self.send_state.send(state);
                }
            }
        }
    }
}
