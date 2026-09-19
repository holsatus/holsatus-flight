use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_time::{Duration, Ticker, with_timeout};

use crate::{
    filters::motor_lin::MotorLin,
    multicopter::attitude_control::MOTORS_MIXED,
    signals::MOTORS_STATE,
    sync::{channel, watch},
    tasks::commander::COMMAD_ARM_VEHICLE,
    types::actuators::MotorOutputs,
};

use params::Reverse;

use crate::{
    abstraction::dshot_group::DshotGroup,
    types::actuators::{DisarmReason, MotorsState},
};

pub mod params {

    #[derive(mav_param::Tree, Clone, Debug)]
    pub struct Params {
        pub rev: Reverse,
        pub timeout_ms: u16,
        pub lin: Linearizer,
    }

    crate::const_default!(
        Params => {
            rev: Reverse::const_default(),
            timeout_ms: 100,
            lin: Linearizer::const_default(),
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

    crate::param_table!(pub static TABLE: Params as "dsht");
}

struct DshotRunner<'a, O> {
    motors: O,
    reverse: [bool; 4],
    timeout: Duration,
    motor_map: MotorLin<f32>,
    recv_channel: channel::Receiver<'a, Message, 1>,
    recv_arm: watch::Receiver<'a, bool>,
    recv_motors: watch::Receiver<'a, [f32; 4]>,
    send_state: watch::Sender<'a, MotorsState>,
}

#[derive(Debug, Clone, PartialEq)]
enum State {
    Disarmed,
    Armed,
}

pub enum Message {
    ReloadParams,
}

static CHANNEL: channel::Channel<Message, 1> = channel::Channel::new();

pub async fn main(output: impl DshotGroup) -> ! {
    let mut state = DshotRunner::new(
        output,
        &*params::TABLE.read().await,
        CHANNEL.receiver(),
        COMMAD_ARM_VEHICLE.receiver(),
        MOTORS_MIXED.receiver(),
        MOTORS_STATE.sender(),
    );

    // Spawn parameter update notifier
    let spawner = SendSpawner::for_current_executor().await;
    if let Ok(task_token) = params_notifier() {
        spawner.spawn(task_token);
    }

    state.run().await
}

#[embassy_executor::task]
async fn params_notifier() -> ! {
    params::TABLE
        .run_notifier(|| CHANNEL.send(Message::ReloadParams))
        .await
}

impl<'a, M: DshotGroup> DshotRunner<'a, M> {
    fn new(
        motors: M,
        params: &params::Params,
        recv_channel: channel::Receiver<'a, Message, 1>,
        recv_arm: watch::Receiver<'a, bool>,
        recv_motors: watch::Receiver<'a, [f32; 4]>,
        send_state: watch::Sender<'a, MotorsState>,
    ) -> Self {
        let timeout = Duration::from_millis(params.timeout_ms as u64);
        let motor_map = MotorLin::new(params.lin.a, params.lin.b, 0.05, 1.0);

        let reverse = [
            params.rev.contains(Reverse::MOTOR_1),
            params.rev.contains(Reverse::MOTOR_2),
            params.rev.contains(Reverse::MOTOR_3),
            params.rev.contains(Reverse::MOTOR_4),
        ];

        Self {
            motors,
            reverse,
            timeout,
            motor_map,
            recv_channel,
            recv_arm,
            recv_motors,
            send_state,
        }
    }

    fn on_params_reload(&mut self, params: &params::Params) {
        self.timeout = Duration::from_millis(params.timeout_ms as u64);
        self.motor_map = MotorLin::new(params.lin.a, params.lin.b, 0.05, 1.0);

        self.reverse = [
            params.rev.contains(Reverse::MOTOR_1),
            params.rev.contains(Reverse::MOTOR_2),
            params.rev.contains(Reverse::MOTOR_3),
            params.rev.contains(Reverse::MOTOR_4),
        ];
    }

    async fn run(&mut self) -> ! {
        let mut state = State::Disarmed;
        self.send_state
            .send(MotorsState::Disarmed(DisarmReason::Uninitialized));

        loop {
            state = match state {
                State::Disarmed => self.run_disarmed().await,
                State::Armed => self.run_armed().await,
            };
        }
    }

    async fn configure_esc(&mut self) {
        self.motors.set_reversed(self.reverse).await;
    }

    async fn run_disarmed(&mut self) -> State {
        let mut esc_configure_ticker = Ticker::every(Duration::from_hz(2));
        let mut zero_throttle_ticker = Ticker::every(Duration::from_hz(100));
        loop {
            match select4(
                self.recv_channel.receive(),
                self.recv_arm.changed(),
                esc_configure_ticker.next(),
                zero_throttle_ticker.next(),
            )
            .await
            {
                Either4::First(message) => match message {
                    Message::ReloadParams => {
                        info!("[dshot_runner] Reloading parameters");
                        let params = params::TABLE.read().await;
                        self.on_params_reload(&params);
                        self.configure_esc().await;
                    }
                },
                Either4::Second(true) => {
                    info!("[dshot_runner] Arming motors as commanded");
                    self.configure_esc().await;
                    return State::Armed;
                }
                Either4::Second(false) => {
                    warn!("[dshot_runner] Disarm commanded while already disarmed");
                }
                Either4::Third(()) => {
                    self.configure_esc().await;
                }
                Either4::Fourth(()) => {
                    self.motors.stop_motors().await;
                }
            }
        }
    }

    async fn run_armed(&mut self) -> State {
        loop {
            match select(
                self.recv_arm.changed(),
                with_timeout(self.timeout, self.recv_motors.changed()),
            )
            .await
            {
                Either::First(false) => {
                    info!("[dshot_runner] Disarming motors as commanded");
                    self.motors.stop_motors().await;

                    let state = MotorsState::Disarmed(DisarmReason::UserCommand);
                    self.send_state.send(state);
                    return State::Disarmed;
                }
                Either::First(true) => {
                    warn!("[dshot_runner] Arm commanded while already armed");
                }
                Either::Second(Err(_)) => {
                    warn!("[dshot_runner] Disarming motors due to timeout");
                    self.motors.stop_motors().await;

                    let state = MotorsState::Disarmed(DisarmReason::Timeout);
                    self.send_state.send(state);
                    return State::Disarmed;
                }
                Either::Second(Ok(mut speeds)) => {
                    speeds = speeds.map(|x| self.motor_map.force_to_command(x));
                    self.motors.set_speeds(speeds).await;
                    let state = MotorsState::Armed(MotorOutputs(speeds));
                    self.send_state.send(state);
                }
            }
        }
    }
}
