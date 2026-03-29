//! Diagnostic binary for the MicoAir H743 flight system.
//!
//! Adds each flight system component incrementally with UART log messages
//! and timed pauses between steps. The last log message visible on UART
//! identifies the step that caused the failure.
//!
//! Flash and open miniterm at 115200 baud. A message appears roughly every
//! 500 ms during startup. If output stops at "step N", the component added
//! in step N+1 is the culprit.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

use common::nalgebra::{UnitQuaternion, Vector3};
use common::signals;
use common::tasks::att_estimator;
use common::tasks::controller_rate;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use common::tasks::eskf::EskfEstimate;
use common::tasks::motor_governor::params;
use common::types::config::DshotConfig;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Timer;
use micoairh743v2::alt_hold::ALTITUDE_SETPOINT;
use micoairh743v2::config::MOTOR_REVERSE_FLAGS;
use micoairh743v2::log as ulog;
use micoairh743v2::resources::{self, UartLogResources};

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
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());
    let r = micoairh743v2::resources::split(p);

    // LED on immediately
    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);

    // UART writer -- must run first so all subsequent log calls are visible
    thread_spawner.spawn(uart_writer_task(r.uart_log).unwrap());

    // Give the UART writer task a chance to initialize before we log anything
    Timer::after_millis(50).await;
    ulog::log("[diag] step 1: board init + UART ok");

    // ------------------------------------------------------------------
    // Step 2: Pre-initialize global signals
    // ------------------------------------------------------------------
    // Use 100 Hz in debug mode so each task has 10 ms per cycle.
    // At 1000 Hz the combined active CPU time of all interrupt-priority
    // tasks exceeds the 1 ms budget in debug (opt-level='z') builds,
    // starving the thread-mode timer and halting UART output.
    signals::CONTROL_FREQUENCY.store(100, Ordering::Relaxed);
    signals::ESKF_ESTIMATE.send(EskfEstimate {
        pos: Vector3::zeros(),
        vel: Vector3::zeros(),
        att: UnitQuaternion::identity(),
        gyr_bias: Vector3::zeros(),
        acc_bias: Vector3::zeros(),
    });
    signals::TRUE_RATE_SP.send([0.0_f32; 3]);
    signals::TRUE_Z_THRUST_SP.send(0.0_f32);
    ulog::log("[diag] step 2: signals pre-initialized");
    Timer::after_millis(50).await;

    // ------------------------------------------------------------------
    // Step 3: Param storage
    // ------------------------------------------------------------------
    thread_spawner.spawn(param_storage_task().unwrap());
    // Direct write to motor params before any task reads them
    params::TABLE.params.write().await.rev = MOTOR_REVERSE_FLAGS;
    ulog::log("[diag] step 3: param_storage spawned, motor params written");
    // Yield long enough for param_storage to enter its receive loop
    Timer::after_millis(100).await;
    ulog::log("[diag] step 3b: param_storage settled");

    // ------------------------------------------------------------------
    // Step 4: Interrupt executors (no tasks yet)
    // ------------------------------------------------------------------
    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    let level_1_spawner = interrupt_executor!(FDCAN1_IT1, P11);
    Timer::after_millis(50).await;
    ulog::log("[diag] step 4: interrupt executors created (no tasks yet)");

    // ------------------------------------------------------------------
    // Step 5: motor_governor at P10
    // (calls TABLE.read() -> param_storage request -> response)
    // ------------------------------------------------------------------
    level_0_spawner
        .spawn(resources::motor_governor_task(r.motors, DshotConfig::Dshot300).unwrap());
    // Wait long enough for the TABLE.read() -> param_storage round-trip
    Timer::after_millis(500).await;
    ulog::log("[diag] step 5: motor_governor spawned + TABLE.read settled");

    // ------------------------------------------------------------------
    // Step 6: imu_reader at P10
    // (does BMI088 init via SPI, then TABLE.read())
    // ------------------------------------------------------------------
    level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());
    Timer::after_millis(500).await;
    ulog::log("[diag] step 6: imu_reader spawned + BMI088 init attempted");

    // ------------------------------------------------------------------
    // Step 7: controller_rate at P10
    // (calls TABLE.read(), then waits for IMU data)
    // ------------------------------------------------------------------
    level_0_spawner.spawn(controller_rate::main().unwrap());
    Timer::after_millis(500).await;
    ulog::log("[diag] step 7: controller_rate spawned + TABLE.read settled");

    // ------------------------------------------------------------------
    // Step 8: att_estimator at P11
    // (waits for IMU or mag data, no TABLE.read)
    // ------------------------------------------------------------------
    ulog::log("[diag] step 8: about to spawn att_estimator");
    Timer::after_millis(50).await; // flush UART before spawn
    level_1_spawner.spawn(att_estimator::main().unwrap());
    Timer::after_millis(200).await;
    ulog::log("[diag] step 8b: att_estimator survived first run");

    // ------------------------------------------------------------------
    // Step 9: alt_hold at thread priority
    // (does DPS310 I2C init, then PI control loop)
    // ------------------------------------------------------------------
    thread_spawner.spawn(resources::alt_hold_task(r.baro).unwrap());
    Timer::after_millis(500).await;
    ulog::log("[diag] step 9: alt_hold spawned + DPS310 init attempted");

    // ------------------------------------------------------------------
    // Step 10: mission_sequencer
    // ------------------------------------------------------------------
    thread_spawner.spawn(mission_sequencer().unwrap());
    ulog::log("[diag] step 10: all tasks spawned -- entering heartbeat");

    // ------------------------------------------------------------------
    // Heartbeat: should run indefinitely if everything is healthy
    // ------------------------------------------------------------------
    loop {
        led_green.toggle();
        ulog::log("[diag] heartbeat");
        Timer::after_secs(2).await;
    }
}

// ------------------------------------------------------------------
// UART log writer (same as flight.rs)
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
            defmt::error!("USART1 init failed");
            loop {
                ulog::CHANNEL.receive().await;
            }
        }
    }
}

// ------------------------------------------------------------------
// Param storage (DummyFlash)
// ------------------------------------------------------------------

const DUMMY_FLASH_SIZE: u32 = 262_144; // sequential_storage needs >= 2 erase sectors

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
