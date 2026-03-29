//! MicoAir H743 -- autonomous flight binary.
//!
//! Architecture:
//!   P10 (FDCAN1_IT0) -- rate controller, IMU reader, motor governor
//!   P11 (FDCAN1_IT1) -- attitude estimator
//!   Thread           -- param storage, alt hold, mission sequencer, UART log
//!
//! Mission sequence (bench-safe with props removed):
//!   1. Wait 2 s for sensor stabilisation
//!   2. Arm motors (motor_governor sends min-throttle for ESC arming)
//!   3. Climb setpoint -> 0.5 m (alt-hold PI)
//!   4. Hover 5 s
//!   5. Descend setpoint -> 0.0 m
//!   6. Wait 5 s then disarm
//!
//! WARNING: remove propellers before bench testing.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write;

use common::nalgebra::{UnitQuaternion, Vector3};
use common::signals;
use common::tasks::att_estimator;
use common::tasks::controller_angle;
use common::tasks::controller_rate;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use common::tasks::eskf::EskfEstimate;
use common::tasks::motor_governor::params;
use common::types::actuators::MotorsState;
use common::types::config::DshotConfig;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Timer;
use micoairh743v2::alt_hold::ALTITUDE_SETPOINT;
use micoairh743v2::config::MOTOR_REVERSE_FLAGS;
use micoairh743v2::log as ulog;
use micoairh743v2::resources::{self, UartLogResources};

/// Helper macro to create an interrupt executor at the given priority.
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::{InterruptExt, Priority};

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            EXECUTOR.on_interrupt()
        }

        spawner
    }};
}

#[embassy_executor::main]
async fn main(thread_spawner: embassy_executor::Spawner) {
    // ------------------------------------------------------------------
    // Board init: PLL1 -> 400 MHz SYSCLK, HCLK = 200 MHz, APB = 100 MHz.
    // SPI2 kernel clock stays on PER (HSI = 64 MHz) so the SPI divisor
    // already chosen gives exactly 8 MHz.
    // ------------------------------------------------------------------
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let r = resources::split(p);

    // Green LED on immediately -- first sign of life before any other init.
    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);

    // ------------------------------------------------------------------
    // UART log writer (async DMA, same setup as sensors.rs test binary).
    // Spawned before anything else so startup messages are captured.
    // ------------------------------------------------------------------
    thread_spawner.spawn(uart_writer_task(r.uart_log).unwrap());

    ulog::log("[flight] board init ok");

    // ------------------------------------------------------------------
    // Pre-initialize global signals so tasks that call Watch::get() or
    // Watch::changed() do not block indefinitely at startup.
    // ------------------------------------------------------------------
    signals::CONTROL_FREQUENCY.store(1000, Ordering::Relaxed);

    signals::ESKF_ESTIMATE.send(EskfEstimate {
        pos: Vector3::zeros(),
        vel: Vector3::zeros(),
        att: UnitQuaternion::identity(),
        gyr_bias: Vector3::zeros(),
        acc_bias: Vector3::zeros(),
    });

    signals::TRUE_RATE_SP.send([0.0_f32; 3]);
    signals::TRUE_Z_THRUST_SP.send(0.0_f32);
    // Level-hover attitude setpoint: identity quaternion = wings level.
    // controller_angle blocks until this is set, so it must come before
    // the interrupt executors start spawning angle_controller.
    signals::TRUE_ATTITUDE_Q_SP.send(UnitQuaternion::identity());

    // ------------------------------------------------------------------
    // Param storage (DummyFlash -- all tables load with const defaults)
    // ------------------------------------------------------------------
    thread_spawner.spawn(param_storage_task().unwrap());

    // ------------------------------------------------------------------
    // Motor reverse-flag override.
    // Write directly to the params RwLock, bypassing the param_storage
    // request/response mechanism. Safe because DummyFlash only returns
    // 0xFF (empty), so LoadTable would restore defaults regardless.
    // Must happen before interrupt executors start to avoid a race with
    // motor_governor reading the table.
    // ------------------------------------------------------------------
    params::TABLE.params.write().await.rev = MOTOR_REVERSE_FLAGS;
    ulog::log("[flight] motor params overridden");

    // ------------------------------------------------------------------
    // Yield to let param_storage_task initialize and enter its receive
    // loop before the interrupt-executor tasks start calling TABLE.read().
    // Without this, motor_governor / controller_rate / imu_reader all
    // fire TABLE.read() at P10 priority before param_storage has had a
    // single poll.  The f405-dev reference uses the same pattern.
    // ------------------------------------------------------------------
    Timer::after_millis(10).await;

    // ------------------------------------------------------------------
    // Interrupt executors
    // ------------------------------------------------------------------
    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    let level_1_spawner = interrupt_executor!(FDCAN1_IT1, P11);

    ulog::log("[flight] executors started");

    // Yield briefly after starting the executors so the executors
    // themselves get a chance to settle before we start spawning.
    Timer::after_millis(1).await;

    // ------------------------------------------------------------------
    // High-priority tasks (P10)
    // ------------------------------------------------------------------
    level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());
    level_0_spawner.spawn(resources::motor_governor_task(r.motors, DshotConfig::Dshot300).unwrap());
    level_0_spawner.spawn(controller_rate::main().unwrap());

    // ------------------------------------------------------------------
    // Medium-priority tasks (P11)
    // ------------------------------------------------------------------
    level_1_spawner.spawn(att_estimator::main().unwrap());
    level_1_spawner.spawn(ahrs_to_eskf_bridge().unwrap());
    level_1_spawner.spawn(controller_angle::main().unwrap());
    level_1_spawner.spawn(angle_to_rate_bridge().unwrap());

    // ------------------------------------------------------------------
    // Thread-priority tasks
    // ------------------------------------------------------------------
    thread_spawner.spawn(resources::alt_hold_task(r.baro).unwrap());
    thread_spawner.spawn(mission_sequencer().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());

    ulog::log("[flight] all tasks spawned");

    // ------------------------------------------------------------------
    // Heartbeat: blink green LED and send UART ping every 2 s.
    // The UART ping means miniterm will show output within 2 s of
    // being opened, regardless of when the startup burst was sent.
    // ------------------------------------------------------------------
    loop {
        led_green.toggle();
        ulog::log("[flight] heartbeat");
        Timer::after_secs(2).await;
    }
}

// ------------------------------------------------------------------
// UART log writer.
// Uses async DMA UART (same approach as sensors.rs, proven on this board).
// Runs at thread priority; drains the log channel.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn uart_writer_task(r: UartLogResources) -> ! {
    bind_interrupts!(struct UartIrqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    });

    match UartTx::new(r.usart, r.tx, r.dma, UartIrqs, UartConfig::default()) {
        Ok(mut uart) => loop {
            let msg = ulog::CHANNEL.receive().await;
            uart.write(msg.as_bytes()).await.ok();
            uart.write(b"\r\n").await.ok();
        },
        Err(_) => {
            defmt::error!("USART1 init failed -- no UART log output");
            loop {
                // Drain the channel so senders never block.
                ulog::CHANNEL.receive().await;
            }
        }
    }
}

// ------------------------------------------------------------------
// Param storage backed by a dummy NorFlash (all-0xFF = empty/erased).
// ------------------------------------------------------------------

const DUMMY_FLASH_SIZE: u32 = 262_144; // 256 KB -- sequential_storage needs >= 2 erase sectors

#[embassy_executor::task]
async fn param_storage_task() -> ! {
    common::tasks::param_storage::entry(DummyFlash, 0..DUMMY_FLASH_SIZE).await
}

struct DummyFlash;

impl common::embedded_storage_async::nor_flash::ErrorType for DummyFlash {
    type Error = core::convert::Infallible;
}

impl common::embedded_storage_async::nor_flash::ReadNorFlash for DummyFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, _offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        bytes.fill(0xFF);
        Ok(())
    }

    fn capacity(&self) -> usize {
        DUMMY_FLASH_SIZE as usize
    }
}

impl common::embedded_storage_async::nor_flash::NorFlash for DummyFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 131_072; // = DUMMY_FLASH_SIZE / 2

    async fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Ok(())
    }
}

// ------------------------------------------------------------------
// Mission sequencer
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn mission_sequencer() -> ! {
    ulog::log("[mission] waiting 2 s for sensor stabilization");
    Timer::after_secs(2).await;

    ulog::log("[mission] arming motors");
    COMMAD_ARM_VEHICLE.send(true);

    Timer::after_secs(3).await;

    ulog::log("[mission] climb setpoint 0.5 m");
    ALTITUDE_SETPOINT.signal(0.5);
    Timer::after_secs(5).await;

    ulog::log("[mission] hovering");
    Timer::after_secs(5).await;

    ulog::log("[mission] descent setpoint 0.0 m");
    ALTITUDE_SETPOINT.signal(0.0);
    Timer::after_secs(5).await;

    ulog::log("[mission] disarming");
    COMMAD_ARM_VEHICLE.send(false);

    ulog::log("[mission] sequence complete");
    loop {
        Timer::after_secs(60).await;
    }
}

// ------------------------------------------------------------------
// Bridge: att_estimator (Madgwick) -> ESKF_ESTIMATE consumed by controller_angle.
//
// att_estimator publishes to AHRS_ATTITUDE_Q; controller_angle reads
// ESKF_ESTIMATE. Without this bridge, ESKF_ESTIMATE only ever holds the
// identity quaternion written at boot, so controller_angle runs once and
// then blocks forever -- no attitude control at all.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn ahrs_to_eskf_bridge() -> ! {
    let mut rcv = signals::AHRS_ATTITUDE_Q.receiver();
    let mut snd = signals::ESKF_ESTIMATE.sender();
    loop {
        let att = rcv.changed().await;
        snd.send(EskfEstimate {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            att,
            gyr_bias: Vector3::zeros(),
            acc_bias: Vector3::zeros(),
        });
    }
}

// ------------------------------------------------------------------
// Bridge: forwards angle-controller roll/pitch output to the rate controller,
// but forces yaw rate setpoint to zero.
//
// controller_angle writes to ANGLE_TO_RATE_SP; controller_rate reads
// TRUE_RATE_SP. signal_router normally connects them but it requires RC
// input and a full control-mode state machine.
//
// Yaw is zeroed here because without a magnetometer there is no absolute
// yaw reference: the ESKF can only integrate gyro drift, so any non-zero
// yaw angle setpoint causes the angle controller to hunt against a moving
// target and oscillate. Roll and pitch are fine because gravity provides
// a stable tilt reference. This is the standard behaviour for compass-less
// drones: angle-hold for roll/pitch, rate-hold (zero rate) for yaw.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn angle_to_rate_bridge() -> ! {
    let mut rcv = signals::ANGLE_TO_RATE_SP.receiver();
    let mut snd = signals::TRUE_RATE_SP.sender();
    loop {
        let [roll, pitch, _yaw] = rcv.changed().await;
        snd.send([roll, pitch, 0.0]);
    }
}

// ------------------------------------------------------------------
// IMU monitor: logs acc + gyro at 1 Hz to UART.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn imu_monitor() -> ! {
    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();
    let mut counter = 0u32;
    loop {
        Timer::after_millis(100).await;
        counter += 1;
        if counter < 10 {
            continue;
        }
        counter = 0;
        let Some(d) = rcv.try_get() else { continue };
        let mut s: heapless::String<96> = heapless::String::new();
        let _ = write!(
            s,
            "[imu] a=[{:.2},{:.2},{:.2}] g=[{:.1},{:.1},{:.1}]",
            d.acc[0], d.acc[1], d.acc[2],
            d.gyr[0], d.gyr[1], d.gyr[2],
        );
        ulog::log(s.as_str());
    }
}

// ------------------------------------------------------------------
// Motor monitor: logs the motor governor state at 1 Hz to UART.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn motor_monitor() -> ! {
    let mut rcv = signals::MOTORS_STATE.receiver();
    loop {
        Timer::after_secs(1).await;
        let Some(state) = rcv.try_get() else { continue };
        let mut s: heapless::String<64> = heapless::String::new();
        match state {
            MotorsState::Disarmed(reason) => {
                let _ = write!(s, "[mtr] disarmed ({:?})", reason);
            }
            MotorsState::Arming => {
                let _ = write!(s, "[mtr] arming");
            }
            MotorsState::ArmedIdle => {
                let _ = write!(s, "[mtr] armed-idle (thr=48)");
            }
            MotorsState::Armed(speeds) => {
                let _ = write!(
                    s,
                    "[mtr] armed [{},{},{},{}]",
                    speeds[0], speeds[1], speeds[2], speeds[3]
                );
            }
        }
        ulog::log(s.as_str());
    }
}
