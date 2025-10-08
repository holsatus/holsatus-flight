use crate::signals as s;
use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker};

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorTest {
    Stop,
    Ramp {
        size: f32,
        rate: f32,
        end: f32,
    },
    Step {
        millis_warmup: u16,
        millis_step: u16,
        end: f32,
    },
    Roll {
        steady_speed: f32,
        millis_steady: u16,
        delta_speed: f32,
        millis_delta: u16,
    },
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum State {
    Start(Instant),
    Main(Instant),
    End(Instant),
    Exit(Instant),
}

pub static SIGNAL: Signal<CriticalSectionRawMutex, MotorTest> = Signal::new();

#[embassy_executor::task]
pub async fn main() {
    const ID: &str = "motor_test";
    info!("{}: Task started", ID);

    // Task outputs
    let mut snd_ctrl_motors = s::CTRL_MOTORS.sender();
    let mut ticker = Ticker::every(Duration::from_hz(1000));

    'infinite: loop {
        let mut test = match select(ticker.next(), SIGNAL.wait()).await {
            // Keep the motors armed
            embassy_futures::select::Either::First(()) => {
                snd_ctrl_motors.send([0.0; 4]);
                continue 'infinite;
            }
            embassy_futures::select::Either::Second(test) => test,
        };
        let mut state = State::Start(Instant::now());
        let mut cmd = 0.0;

        'testing: loop {
            ticker.next().await;

            if let Some(new_test) = SIGNAL.try_take() {
                test = new_test;
            }

            match &test {
                MotorTest::Stop => {
                    snd_ctrl_motors.send([0.0, 0.0, 0.0, 0.0]);
                    break 'testing;
                }
                MotorTest::Ramp { size, rate, end } => match &mut state {
                    State::Start(time) => {
                        if time.elapsed() < Duration::from_millis(500) {
                            snd_ctrl_motors.send([0.001; 4]);
                        } else if time.elapsed() < Duration::from_millis(1000) {
                            snd_ctrl_motors.send([*size; 4]);
                        } else if time.elapsed() < Duration::from_millis(1500) {
                            snd_ctrl_motors.send([0.001; 4]);
                        } else {
                            state = State::Main(Instant::now());
                        }
                    }
                    State::Main(time) => {
                        cmd = time.elapsed().as_micros() as f32 * 1e-6 * rate;
                        if cmd < *end {
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            state = State::End(Instant::now());
                        }
                    }
                    State::End(time) => {
                        if time.elapsed() < Duration::from_millis(500) {
                            snd_ctrl_motors.send([*end; 4]);
                        } else {
                            state = State::Exit(Instant::now());
                        }
                    }
                    State::Exit(time) => {
                        cmd = end - time.elapsed().as_micros() as f32 * 2e-6;
                        if cmd > 0.0 {
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            snd_ctrl_motors.send([0.0; 4]);
                            break 'testing;
                        }
                    }
                },
                MotorTest::Step {
                    millis_warmup,
                    millis_step,
                    end,
                } => match &mut state {
                    State::Start(time) => {
                        if time.elapsed() < Duration::from_millis(500) {
                            cmd = 0.001;
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            cmd = 0.1;
                            state = State::Main(Instant::now());
                        }
                    }
                    State::Main(time) => {
                        if time.elapsed() < Duration::from_millis(*millis_warmup as u64) {
                            snd_ctrl_motors.send([0.05; 4]);
                        } else if time.elapsed()
                            < Duration::from_millis(*millis_warmup as u64 + *millis_step as u64)
                        {
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            cmd += 0.1;
                            if cmd <= *end + 0.001 {
                                state = State::Main(Instant::now());
                            } else {
                                state = State::End(Instant::now());
                            }
                        }
                    }
                    State::End(time) => {
                        state = State::Exit(*time);
                    }
                    State::Exit(time) => {
                        cmd = end - time.elapsed().as_micros() as f32 * 2e-6;
                        if cmd > 0.0 {
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            snd_ctrl_motors.send([0.0; 4]);
                            break 'testing;
                        }
                    }
                },
                MotorTest::Roll {
                    steady_speed,
                    millis_steady,
                    delta_speed,
                    millis_delta,
                } => match &state {
                    State::Start(time) => {
                        if time.elapsed() < Duration::from_millis(*millis_steady as u64) {
                            snd_ctrl_motors.send([*steady_speed; 4]);
                        } else {
                            state = State::Main(Instant::now());
                        }
                    }
                    State::Main(time) => {
                        if time.elapsed() < Duration::from_millis(*millis_delta as u64) {
                            let speeds = [
                                *steady_speed + delta_speed,
                                *steady_speed - delta_speed,
                                *steady_speed + delta_speed,
                                *steady_speed - delta_speed,
                            ];
                            snd_ctrl_motors.send(speeds);
                        } else {
                            state = State::End(Instant::now());
                        }
                    }
                    State::End(time) => {
                        state = State::Exit(*time);
                    }
                    State::Exit(time) => {
                        cmd = *steady_speed - time.elapsed().as_micros() as f32 * 2e-6;
                        if cmd > 0.0 {
                            snd_ctrl_motors.send([cmd; 4]);
                        } else {
                            snd_ctrl_motors.send([0.0; 4]);
                            break 'testing;
                        }
                    }
                },
            }
        }
    }
}
