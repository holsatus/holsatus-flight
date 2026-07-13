//! MicoAir H743 -- landing-gear leveling helper.
//!
//! Streams the drone's tilt (derived from the raw accelerometer) over UART
//! at 4 Hz so you can iteratively adjust zip ties / legs and watch the
//! numbers converge toward zero in real time.
//!
//! Usage:
//!   1. Flash this binary: `make flash-release BIN=level_check`
//!   2. Open miniterm: `python3 -m serial.tools.miniterm PORT 115200`
//!   3. Put drone on its landing gear, hands off.
//!   4. Watch the `pitch=...  roll=...` line update every 250 ms.
//!   5. Adjust zip ties / leg heights until both are within +/- 1 deg.
//!   6. Flash flight.rs for the actual flight test.
//!
//! No motors, no SD card, no mission. Safe to run with props on / battery
//! connected -- nothing spins. Pitch/roll computed from raw accel only
//! (no gyro integration, no Madgwick, no cal) so there is no attitude
//! estimator lag -- each reading is an instantaneous tilt.
//!
//! Sign convention (drone body frame, arrow-forward = nose):
//!   pitch > 0 = nose up,  pitch < 0 = nose down
//!   roll  > 0 = right wing down,  roll < 0 = left wing down
//! When the drone body is level with gravity, both read 0.
//!
//! NOTE on axis mapping: the BMI088 on the MicoAir H743v2 is mounted
//! rotated 90 deg relative to the FC's arrow-forward convention.
//! Empirically (verified with user tilt test 2026-04-21):
//!   chip +X points to the drone's LEFT
//!   chip +Y points to the drone's FORWARD (nose)
//!   chip +Z points UP (same as drone -Z in NED)
//! so  drone_ax (forward) = chip_ay
//!     drone_ay (right)   = -chip_ax
//!     drone_az (up)      = chip_az  (both point up; drone body Z-down is
//!                                    a separate firmware-wide convention)
//! This file intentionally reports in the arrow-forward frame so the
//! numbers match the user's physical mental model. The rest of the
//! firmware currently operates in raw-chip frame; that's a separate
//! known issue (`imu_reader::params::rot` is identity; should be set
//! to a 90 deg rotation).

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write;

use common::signals;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use micoairh743v2::log as ulog;
use micoairh743v2::resources::{self, UartLogResources};
use micoairh743v2::resources::UartLogIrqs;

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

    let r = resources::split(p);

    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);
    let _led_blue = Output::new(r.leds.blue, Level::Low, Speed::Low);
    let _led_red = Output::new(r.leds.red, Level::Low, Speed::Low);

    thread_spawner.spawn(uart_only_writer(r.uart_log).unwrap());

    ulog::log("[level] board init ok");
    ulog::log("[level] put drone on its gear, hands off, watch pitch/roll");

    signals::CONTROL_FREQUENCY.store(1000, Ordering::Relaxed);

    thread_spawner.spawn(param_storage_task().unwrap());

    // Yield so param_storage_task starts its receive loop before imu_reader
    // fires its first TABLE.read().
    Timer::after_millis(10).await;

    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());

    // Drop unused resources (no motors, no SD, no baro, no RC, no mtf01).
    drop(r.motors);
    drop(r.sdmmc);
    drop(r.baro);
    drop(r.mtf01);
    drop(r.rc);
    drop(r.battery);

    ulog::log("[level] waiting 2s for IMU to start publishing...");
    Timer::after_secs(2).await;

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();
    let mut avg_ax = 0.0_f32;
    let mut avg_ay = 0.0_f32;
    let mut avg_az = 9.81_f32;
    const ALPHA: f32 = 0.2;

    loop {
        let d = rcv.changed().await;
        // Low-pass the raw accel so the output isn't flickery from motor-
        // off vibration, ambient building noise, etc. This is NOT the full
        // IMU calibration -- we're intentionally showing the raw tilt so
        // the user sees the true orientation, not a cal-biased reading.
        avg_ax = ALPHA * d.acc[0] + (1.0 - ALPHA) * avg_ax;
        avg_ay = ALPHA * d.acc[1] + (1.0 - ALPHA) * avg_ay;
        avg_az = ALPHA * d.acc[2] + (1.0 - ALPHA) * avg_az;

        // Chip readings (raw BMI088 frame, Z up).
        let cx = avg_ax;
        let cy = avg_ay;
        let cz = avg_az;

        // Tilt-from-gravity in drone arrow-forward frame. Formulas are
        // derived for Z-up body convention; the chip-to-drone axis swap
        // is folded in (see file header):
        //   pitch (nose up > 0)   = rotation about drone Y = rotation about chip X
        //   roll  (right down >0) = rotation about drone X = rotation about chip Y
        let pitch_deg = libm::atan2f(-cy, libm::sqrtf(cx * cx + cz * cz)).to_degrees();
        let roll_deg = libm::atan2f(cx, cz).to_degrees();

        // Report every ~250 ms (the receiver fires at ~1 kHz, so throttle).
        static mut LAST_LOG_MS: u64 = 0;
        let now_ms = embassy_time::Instant::now().as_millis();
        // Safety: single-threaded access to LAST_LOG_MS from this main-task
        // loop only. No other context touches it.
        let should_log = unsafe {
            if now_ms - LAST_LOG_MS >= 250 {
                LAST_LOG_MS = now_ms;
                true
            } else {
                false
            }
        };

        if should_log {
            led_green.toggle();
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[level] pitch={:+.2} roll={:+.2}  chip_ax={:+.2} ay={:+.2} az={:+.2}",
                pitch_deg, roll_deg, cx, cy, cz,
            );
            ulog::log(s.as_str());
        }
    }
}

// ------------------------------------------------------------------
// Minimal UART-only log writer (no SD card, no logging to disk).
// Same DMA bindings as flight.rs but without the FS init.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn uart_only_writer(r: UartLogResources) -> ! {
    let mut uart = UartTx::new(r.usart, r.tx, r.dma, UartLogIrqs, UartConfig::default()).ok();

    loop {
        let msg = ulog::CHANNEL.receive().await;
        if let Some(ref mut u) = uart {
            u.write(msg.as_bytes()).await.ok();
            u.write(b"\r\n").await.ok();
        }
    }
}

// ------------------------------------------------------------------
// Param storage (dummy flash, same as flight.rs -- imu_reader reads
// its cal params from this table on first startup).
// ------------------------------------------------------------------

const DUMMY_FLASH_SIZE: u32 = 262_144;

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
    const ERASE_SIZE: usize = 131_072;
    async fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Ok(())
    }
}
