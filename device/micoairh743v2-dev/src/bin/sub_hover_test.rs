//! MicoAir H743 -- sub-liftoff vibration / signal-chain test.
//!
//! Runs the full flight-control stack (IMU -> Madgwick -> rate PID -> mixer ->
//! motor governor -> DShot) with a mission that commands up to ~85 percent of
//! hover thrust -- below the typical friction-limited liftoff threshold on a
//! table. The drone should stay on the ground throughout. The test exists to
//! catch the failure modes that killed D000297/306/308 (prop-wash vibration
//! corrupting Madgwick, motor-sign errors, mixer runaway) without risking a
//! crash.
//!
//! Mission sequence:
//!   1. Wait 5 s for IMU calibration and Madgwick settling
//!   2. Abort if SD card is missing (same LED pattern as flight.rs)
//!   3. Arm motors (ESC arming sequence via motor_governor)
//!   4. Ramp thrust 0 -> 80 percent hover over 5 s
//!   5. Hold for 25 s  <-- watch telemetry here
//!   6. Ramp down to 0 over 3 s
//!   7. Disarm
//!
//! A gyro-runaway autoabort task monitors all three axes and disarms the
//! moment |gyro| > 5 rad/s for 50 consecutive samples (50 ms at 1 kHz).
//! This catches a divergent rate loop before the drone can build up
//! enough momentum to break restraint.
//!
//! SAFETY:
//!   - Physically restrain the drone before running. Even at sub-liftoff
//!     thrust, a mixer runaway from a sign error can skitter it off a table.
//!   - Keep a finger on the kill switch (or rip the battery) for the whole
//!     25 s hold.
//!
//! What to watch in the log:
//!   - Gyro > 15 deg/s sustained          -> frame or prop resonance
//!   - Roll/pitch estimate drift > 4 deg  -> Madgwick corrupted by vibration
//!   - Motor spread > 150 DShot at level  -> controller polarity issue
//!   - [mtr] disarmed <reason>            -> motor governor tripped; investigate

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write;

use common::nalgebra::{UnitQuaternion, Vector3};
use common::signals;
use common::tasks::att_estimator;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use common::tasks::controller_angle;
use common::tasks::controller_rate;
use common::tasks::eskf::EskfEstimate;
use common::types::actuators::MotorsState;
use common::types::config::DshotConfig;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use micoairh743v2::log as ulog;
use micoairh743v2::resources::UartLogIrqs;
use micoairh743v2::resources::{self, BatteryResources, SdmmcLogResources, UartLogResources};
use micoairh743v2::sdlog::SdmmcResources;

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
    // Red + blue kept off; re-used below for the "SD absent" abort pattern if
    // uart_writer_task reports the card could not be mounted.
    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);
    let mut led_blue = Output::new(r.leds.blue, Level::Low, Speed::Low);
    let mut led_red = Output::new(r.leds.red, Level::Low, Speed::Low);

    // ------------------------------------------------------------------
    // UART log writer (async DMA, same setup as sensors.rs test binary).
    // Spawned before anything else so startup messages are captured.
    // ------------------------------------------------------------------
    thread_spawner.spawn(uart_writer_task(r.uart_log, r.sdmmc).unwrap());

    ulog::log("[sub] board init ok");

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

    override_motor_params().await;
    override_pid_gains().await;

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

    ulog::log("[sub] executors started");

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

    let _battery_mv = read_battery_mv(r.battery);
    // baro / mtf01 resources deliberately unused: this binary neither holds
    // altitude nor uses optical flow -- thrust is commanded by the mission
    // directly so we can observe the raw rate loop under vibration.
    let _ = r.baro;
    let _ = r.mtf01;

    // ------------------------------------------------------------------
    // Thread-priority tasks
    // ------------------------------------------------------------------
    thread_spawner.spawn(flip_kill().unwrap());
    thread_spawner.spawn(gyro_runaway_kill().unwrap());
    thread_spawner.spawn(micoairh743v2::rc_kill::rc_kill_task(r.rc).unwrap());
    thread_spawner.spawn(sub_hover_mission().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());

    ulog::log("[sub] all tasks spawned");

    apply_imu_calibration().await;

    // ------------------------------------------------------------------
    // SD-card gate.
    //
    // uart_writer_task publishes its SD mount outcome to ulog::SD_MOUNTED:
    //   255 = still running the 3x retry (max ~1.5 s) + FAT mount
    //     1 = ready, log file open
    //     0 = card missing / FAT mount failed
    //
    // If the card never mounts, motors must never arm (the mission task also
    // checks this) and we latch into a distinct three-blue-one-red LED
    // pattern so the pilot knows the drone will not fly without a card.
    // ------------------------------------------------------------------
    {
        let wait_start = embassy_time::Instant::now();
        while ulog::SD_MOUNTED.load(Ordering::Relaxed) == 255
            && wait_start.elapsed().as_millis() < 5_000
        {
            Timer::after_millis(100).await;
        }
    }
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[sub] SD ABORT -- motors will not arm (3x blue + 1x red forever)");
        led_green.set_low();
        loop {
            for _ in 0..3 {
                led_blue.set_high();
                Timer::after_millis(150).await;
                led_blue.set_low();
                Timer::after_millis(150).await;
            }
            led_red.set_high();
            Timer::after_millis(400).await;
            led_red.set_low();
            Timer::after_millis(600).await;
        }
    }

    // ------------------------------------------------------------------
    // Heartbeat: blink green LED, report DShot rate every 2 s.
    // DShot rate = frames transmitted by the P10 critical path per second.
    // Expected ~1000 Hz. A sustained drop below ~950 indicates contention.
    // ------------------------------------------------------------------
    let mut prev_tx =
        micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
    loop {
        led_green.toggle();
        Timer::after_secs(2).await;
        let cur_tx =
            micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
        let rate = (cur_tx - prev_tx) / 2; // frames per second (sampled over 2s)
        prev_tx = cur_tx;
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = write!(s, "[sub] hb dshot={}Hz", rate);
        ulog::log(s.as_str());
    }
}

/// Read battery voltage once via blocking ADC (call before motors spin).
fn read_battery_mv(r: BatteryResources) -> u32 {
    use embassy_stm32::adc::{Adc, SampleTime};
    let mut adc = Adc::new(r.adc);
    let mut pin_v = r.pin_v;
    let raw = adc.blocking_read(&mut pin_v, SampleTime::CYCLES64_5);
    const V_DIV: u32 = 21;
    const ADC_FULL: u32 = 65535;
    const VREF_MV: u32 = 3300;
    let mv = (raw as u32 * VREF_MV * V_DIV) / ADC_FULL;
    let mut s: heapless::String<48> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(&mut s, format_args!("[bat] voltage={} mV", mv));
    ulog::log(s.as_str());
    mv
}

// ------------------------------------------------------------------
// Parameter overrides -- called once at startup before executors start.
// ------------------------------------------------------------------

/// Motor governor: reverse flags and timeout.
async fn override_motor_params() {
    use common::tasks::motor_governor::params;
    use micoairh743v2::config::MOTOR_REVERSE_FLAGS;

    let mut p = params::TABLE.params.write().await;
    p.rev = MOTOR_REVERSE_FLAGS;
    p.timeout_ms = 500; // Watch::changed() can stall when mixer output saturates
    drop(p);

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[sub] rev=0x{:04X} timeout=500ms",
        MOTOR_REVERSE_FLAGS.bits()
    );
    ulog::log(s.as_str());
}

/// Rate and angle PID gains for this frame.
///
/// Reasoning, in order of importance:
///
/// 1. kp = 0.05 roll/pitch, 0.04 yaw is handheld-validated (D000278/283/295).
///    The 10x-scaled holsatus-sim mid-band (kp=0.004-0.0045 roll/pitch,
///    kp=0.0035 yaw) maps to the same range, so the sim and handheld tests
///    agree on the kp scale.
///
/// 2. kd = 0.015 flipped the drone in D000308/309. The effective rate-PID
///    D-gain at that setting was kd*kp/dt = 0.015 * 0.05 / 0.001 = 0.75.
///    In the hardened sim (pipeline latency, ground effect, 5% vibration,
///    asymmetric motor lag, battery sag), D-gains above ~0.60 tumble and
///    the mid-band passing D-gain is ~0.08. We pick kd to match the sim
///    mid-band D-gain at the current kp:
///       kd = D_gain * dt / kp = 0.08 * 0.001 / 0.05 = 0.0016 -> round to 0.002.
///    That is 7.5x below the flip threshold and supplies genuine phase
///    lead against the ~2 ms FC pipeline delay (DShot frame + ESC compute).
///
/// 3. ki = 0 on every axis. The rate-PID integrator is gated by
///    CALM_THRESHOLD = 1.0 rad/s in common/src/filters/rate_pid.rs and
///    therefore never accumulates during active flight. Angle-controller
///    ki causes windup whenever something physically constrains the
///    airframe (hand, string), and is only safe once free hover is proven.
async fn override_pid_gains() {
    use common::tasks::controller_rate;

    let mut r = controller_rate::params::TABLE.params.write().await;
    r.x.kp = 0.05;
    r.x.ki = 0.0;
    r.x.kd = 0.002;
    r.y.kp = 0.05;
    r.y.ki = 0.0;
    r.y.kd = 0.002;
    r.z.kp = 0.04;
    r.z.ki = 0.0;
    r.z.kd = 0.001;
    drop(r);

    // Angle controller: ki adds integral to eliminate steady-state offset.
    // Defaults: kp=15, ki=0, kd=0 for roll/pitch.
    // NOTE: ki > 0 causes windup when ANY external constraint prevents correction
    // (hand, string, stick). Only safe in true free flight where the drone can
    // freely rotate to zero the error. Disabled until free hover is proven stable.
    // {
    //     use common::tasks::controller_angle;
    //     let mut a = controller_angle::params::TABLE.params.write().await;
    //     a.roll.ki = 0.5;
    //     a.pitch.ki = 0.5;
    //     drop(a);
    // }

    ulog::log("[sub] PID gains overridden");
}

/// Wait for Madgwick AHRS to settle after IMU calibration.
/// Monitors roll/pitch variance over a sliding window and returns once
/// attitude is stable. Logs the final attitude so a human reader can
/// verify the surface was reasonably level.
///
/// IMPORTANT: if Madgwick takes more than ~10 seconds to settle, that
/// typically means the drone is being moved or vibrating -- in that case
/// the returned attitude is unreliable and the flight should be aborted.
async fn wait_for_ahrs_ready() {
    use signals::AHRS_ATTITUDE_Q;

    const WINDOW_N: usize = 50; // 500ms at 100 Hz sample rate
    const STABLE_THRESHOLD_DEG: f32 = 0.3; // peak-to-peak roll/pitch range
    const MAX_WAIT_MS: u64 = 10_000;

    let mut rcv = AHRS_ATTITUDE_Q.receiver();
    let mut roll_buf = [0.0_f32; WINDOW_N];
    let mut pitch_buf = [0.0_f32; WINDOW_N];
    let mut idx: usize = 0;
    let mut filled = 0usize;

    ulog::log("[ahrs] waiting for attitude to settle...");
    let start = embassy_time::Instant::now();

    loop {
        // Sample at ~100 Hz via the Watch receiver. changed().await yields
        // whenever att_estimator publishes a new value (100-500 Hz depending
        // on control frequency).
        let q = rcv.changed().await;
        let (r, p, _y) = q.euler_angles();
        roll_buf[idx] = r.to_degrees();
        pitch_buf[idx] = p.to_degrees();
        idx = (idx + 1) % WINDOW_N;
        if filled < WINDOW_N {
            filled += 1;
        }

        if filled == WINDOW_N {
            let r_min = roll_buf.iter().cloned().fold(f32::INFINITY, f32::min);
            let r_max = roll_buf.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            let p_min = pitch_buf.iter().cloned().fold(f32::INFINITY, f32::min);
            let p_max = pitch_buf.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            let r_range = r_max - r_min;
            let p_range = p_max - p_min;

            if r_range < STABLE_THRESHOLD_DEG && p_range < STABLE_THRESHOLD_DEG {
                let r_mean: f32 = roll_buf.iter().sum::<f32>() / WINDOW_N as f32;
                let p_mean: f32 = pitch_buf.iter().sum::<f32>() / WINDOW_N as f32;
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "[ahrs] ready: r={:.2} p={:.2} range=({:.2},{:.2})",
                    r_mean, p_mean, r_range, p_range
                );
                ulog::log(s.as_str());
                return;
            }
        }

        if start.elapsed().as_millis() >= MAX_WAIT_MS {
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[ahrs] WARNING: not settled after {}ms, continuing anyway",
                MAX_WAIT_MS
            );
            ulog::log(s.as_str());
            return;
        }
    }
}

/// Runtime IMU calibration: average 2000 accel+gyro samples (~2 seconds)
/// while the drone sits level and still. Writes bias and axis scale to
/// imu_reader params. Must be called after imu_reader task is running.
async fn apply_imu_calibration() {
    use common::tasks::imu_reader;

    const N: u32 = 2000;
    let mut rcv = common::signals::RAW_MULTI_IMU_DATA[0].receiver();

    ulog::log("[cal] hold level and still (2s)...");

    // Wait for first reading to ensure IMU is producing data.
    rcv.changed().await;

    let mut sum_acc = [0.0_f64; 3];
    let mut sum_gyr = [0.0_f64; 3];
    for _ in 0..N {
        let d = rcv.changed().await;
        sum_acc[0] += d.acc[0] as f64;
        sum_acc[1] += d.acc[1] as f64;
        sum_acc[2] += d.acc[2] as f64;
        sum_gyr[0] += d.gyr[0] as f64;
        sum_gyr[1] += d.gyr[1] as f64;
        sum_gyr[2] += d.gyr[2] as f64;
    }

    let n = N as f64;
    let acc_bias = [
        (sum_acc[0] / n) as f32,
        (sum_acc[1] / n) as f32,
        (sum_acc[2] / n) as f32 - common::consts::GRAVITY,
    ];
    let gyr_bias = [
        (sum_gyr[0] / n) as f32,
        (sum_gyr[1] / n) as f32,
        (sum_gyr[2] / n) as f32,
    ];

    {
        let mut p = imu_reader::params::TABLE.params.write().await;
        p.cal_acc.bias = acc_bias;
        p.cal_gyr.bias = gyr_bias;
        p.cal_acc.scale = [1.0, -1.0, -1.0];
        p.cal_gyr.scale = [1.0, -1.0, -1.0];
    }

    imu_reader::CHANNEL[0]
        .sender()
        .send(imu_reader::Message::ReloadParams)
        .await;

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[cal] acc=[{:.3},{:.3},{:.3}]",
        acc_bias[0], acc_bias[1], acc_bias[2]
    );
    ulog::log(s.as_str());

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[cal] gyr=[{:.4},{:.4},{:.4}]",
        gyr_bias[0], gyr_bias[1], gyr_bias[2]
    );
    ulog::log(s.as_str());
}

// ------------------------------------------------------------------
// UART + SD card log writer.
// Drains the log channel, writes each message to UART1 and (if the SD
// card is mounted) appends it to a text file on the FAT filesystem.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn uart_writer_task(r: UartLogResources, sd: SdmmcLogResources) -> ! {
    use block_device_adapters::BufStream;
    use core::fmt::Write as FmtWrite;
    use embedded_fatfs::{FileSystem, FsOptions};
    use embedded_io_async_061::Write as _;

    let mut uart = UartTx::new(r.usart, r.tx, r.dma, UartLogIrqs, UartConfig::default()).ok();

    // ── SD card setup (best-effort -- logging continues on UART if SD fails) ──
    let mut device = SdmmcResources {
        periph: sd.periph,
        clk: sd.clk,
        cmd: sd.cmd,
        d0: sd.d0,
        d1: sd.d1,
        d2: sd.d2,
        d3: sd.d3,
    }
    .setup();

    let sd_ok = {
        let mut ok = false;
        for _ in 0u8..3 {
            if device.try_reset().await.is_ok() {
                ok = true;
                break;
            }
            Timer::after_millis(500).await;
        }
        ok
    };

    if !sd_ok {
        // No SD -- publish status, log to UART only, never let motors arm.
        ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
        ulog::log("[sd] NOT FOUND (tried 3x) -- motors will NOT arm");
        loop {
            let msg = ulog::CHANNEL.receive().await;
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
        }
    }

    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
            ulog::log("[sd] reset OK but FAT mount FAILED -- motors will NOT arm");
            loop {
                let msg = ulog::CHANNEL.receive().await;
                if let Some(ref mut u) = uart {
                    u.write(msg.as_bytes()).await.ok();
                    u.write(b"\r\n").await.ok();
                }
            }
        }
    };

    // Find next session directory.
    let mut session_idx: u32 = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        if entry.is_dir() {
            let name = entry.short_file_name_as_bytes();
            if name.len() == 7 && (name[0] == b'D' || name[0] == b'd') {
                if let Some(idx) = name[1..].iter().try_fold(0u32, |acc, &b| {
                    if b >= b'0' && b <= b'9' {
                        Some(acc * 10 + (b - b'0') as u32)
                    } else {
                        None
                    }
                }) {
                    session_idx = session_idx.max(idx);
                }
            }
        }
    }
    session_idx += 1;

    let mut dir_name: heapless::String<8> = heapless::String::new();
    let session_dir = loop {
        dir_name.clear();
        let _ = FmtWrite::write_fmt(&mut dir_name, format_args!("D{:06}", session_idx));
        match fs.root_dir().create_dir(dir_name.as_str()).await {
            Ok(d) => break d,
            Err(embedded_fatfs::Error::AlreadyExists) => {
                session_idx += 1;
            }
            Err(_) => loop {
                let msg = ulog::CHANNEL.receive().await;
                if let Some(ref mut u) = uart {
                    u.write(msg.as_bytes()).await.ok();
                    u.write(b"\r\n").await.ok();
                }
            },
        }
    };

    let mut file_name: heapless::String<12> = heapless::String::new();
    let _ = FmtWrite::write_fmt(&mut file_name, format_args!("000001.LOG"));
    let mut file = match session_dir.create_file(file_name.as_str()).await {
        Ok(f) => f,
        Err(_) => {
            ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
            ulog::log("[sd] create_file failed -- motors will NOT arm");
            loop {
                let msg = ulog::CHANNEL.receive().await;
                if let Some(ref mut u) = uart {
                    u.write(msg.as_bytes()).await.ok();
                    u.write(b"\r\n").await.ok();
                }
            }
        }
    };

    // SD card fully mounted and log file open. Motors may now arm.
    ulog::SD_MOUNTED.store(1, Ordering::Relaxed);
    let mut s: heapless::String<32> = heapless::String::new();
    let _ = FmtWrite::write_fmt(
        &mut s,
        format_args!("[sd] mounted -> {}/000001.LOG", dir_name.as_str()),
    );
    ulog::log(s.as_str());

    let mut flush_counter: u16 = 0;
    loop {
        while let Ok(msg) = ulog::CRITICAL_CHANNEL.try_receive() {
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
            file.write_all(msg.as_bytes()).await.ok();
            file.write_all(b"\r\n").await.ok();
            file.flush().await.ok();
        }

        use embassy_futures::select::{select, Either};
        let msg = match select(ulog::CRITICAL_CHANNEL.receive(), ulog::CHANNEL.receive()).await {
            Either::First(m) => {
                if let Some(ref mut u) = uart {
                    u.write(m.as_bytes()).await.ok();
                    u.write(b"\r\n").await.ok();
                }
                file.write_all(m.as_bytes()).await.ok();
                file.write_all(b"\r\n").await.ok();
                file.flush().await.ok();
                continue;
            }
            Either::Second(m) => m,
        };

        if !ulog::is_high_rate_telemetry(msg.as_str()) {
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
        }
        file.write_all(msg.as_bytes()).await.ok();
        file.write_all(b"\r\n").await.ok();

        flush_counter += 1;
        if flush_counter >= 5 {
            flush_counter = 0;
            file.flush().await.ok();
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

/// Sub-liftoff vibration test mission.
///
/// Ramps thrust to 85 percent of hover, holds for 25 s, then ramps down.
/// Level attitude setpoint is left as identity throughout, so roll/pitch
/// rate PIDs are actively trying to hold zero rate against any physical
/// disturbance. That is where vibration-coupled Madgwick drift or a
/// motor-sign error would show up as growing rate_sp / PID output even
/// though the drone is not moving.
#[embassy_executor::task]
async fn sub_hover_mission() -> ! {
    ulog::log("[sub] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    // SD card must be mounted before arming -- no telemetry, no spin-up.
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[sub] ABORT: no SD card, not arming");
        loop {
            Timer::after_secs(60).await;
        }
    }

    wait_for_ahrs_ready().await;

    // Wait for RC link: the kill switch must be functional before any motor
    // spins. rc_kill_task sets RC_LINK_READY once it has parsed its first
    // valid CRSF packet and recorded baseline SE/SF positions.
    {
        ulog::log("[sub] waiting for RC link (30 s timeout)...");
        let start = embassy_time::Instant::now();
        const RC_WAIT_MS: u64 = 30_000;
        while !micoairh743v2::rc_kill::RC_LINK_READY.load(core::sync::atomic::Ordering::Relaxed) {
            if start.elapsed().as_millis() >= RC_WAIT_MS {
                ulog::log("[sub] ABORT: no RC link after 30 s -- motors will NOT arm");
                loop {
                    Timer::after_secs(60).await;
                }
            }
            Timer::after_millis(100).await;
        }
        ulog::log("[sub] RC link established");
    }

    ulog::log("[sub] arming motors (RESTRAIN THE DRONE)");
    COMMAD_ARM_VEHICLE.send(true);
    Timer::after_secs(3).await;

    // 80 percent of hover thrust. Last restrained-only thrust step before
    // either (a) on-hardware PID sweep or (b) careful free flight. D000015
    // at 50% showed motor spread peak 127, converging to 30; vibration
    // envelope < 0.1 m/s^2. Controller math exact against angle_kp=15.
    const HOVER_THRUST: f32 = 4.50;
    const TEST_THRUST: f32 = HOVER_THRUST * 0.80;
    const RAMP_UP_S: u64 = 5;
    const HOLD_S: u64 = 25;
    const RAMP_DN_S: u64 = 3;

    ulog::log("[sub] P0: ramp 0 -> 80% hover (5s)");
    let start = embassy_time::Instant::now();
    loop {
        let elapsed = start.elapsed().as_millis();
        if elapsed >= RAMP_UP_S * 1000 {
            break;
        }
        let frac = elapsed as f32 / (RAMP_UP_S * 1000) as f32;
        signals::TRUE_Z_THRUST_SP.send(TEST_THRUST * frac);
        Timer::after_millis(20).await;
    }

    ulog::log("[sub] P1: hold 80% hover (25s) -- watch gyro / motor spread");
    signals::TRUE_Z_THRUST_SP.send(TEST_THRUST);
    Timer::after_secs(HOLD_S).await;

    ulog::log("[sub] P2: ramp down (3s)");
    let dstart = embassy_time::Instant::now();
    loop {
        let elapsed = dstart.elapsed().as_millis();
        if elapsed >= RAMP_DN_S * 1000 {
            break;
        }
        let frac = 1.0 - elapsed as f32 / (RAMP_DN_S * 1000) as f32;
        signals::TRUE_Z_THRUST_SP.send(TEST_THRUST * frac);
        Timer::after_millis(20).await;
    }
    signals::TRUE_Z_THRUST_SP.send(0.0);

    Timer::after_secs(1).await;
    ulog::log("[sub] disarming");
    COMMAD_ARM_VEHICLE.send(false);

    ulog::log("[sub] test complete");
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
// Yaw is zeroed because without a magnetometer there is no absolute
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
// High-rate telemetry monitor (100 Hz) for PID analysis.
// Alternates between two CSV line types each cycle:
//   A: t,ax,ay,az,gx,gy,gz,m0,m1,m2,m3
//   B: t,rsp0,rsp1,rsp2,pp0,ip0,pp1,ip1,pp2,ip2,thr
// where rspN = rate setpoint, ppN/ipN = PID P/I output per axis
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn imu_monitor() -> ! {
    use common::tasks::controller_rate::{RATE_PID_TERMS, RATE_REF_FILTERED};

    let mut imu_rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();
    let mut mtr_rcv = signals::MOTORS_STATE.receiver();
    let mut att_rcv = signals::AHRS_ATTITUDE_Q.receiver();
    let mut pid_rcv = RATE_PID_TERMS.receiver();
    let mut ref_rcv = RATE_REF_FILTERED.receiver();
    let mut thr_rcv = signals::TRUE_Z_THRUST_SP.receiver();

    let mut cycle: u8 = 0;

    loop {
        Timer::after_millis(10).await; // 100 Hz

        let t = embassy_time::Instant::now().as_millis();

        if cycle == 0 {
            // Line A: IMU + motors
            let Some(d) = imu_rcv.try_get() else { continue };
            let (m0, m1, m2, m3) = match mtr_rcv.try_get() {
                Some(MotorsState::Armed(s)) => (s[0], s[1], s[2], s[3]),
                Some(MotorsState::Arming) => (0u16, 0, 0, 0),
                _ => (0u16, 0, 0, 0),
            };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "A,{},{:.1},{:.1},{:.1},{:.2},{:.2},{:.2},{},{},{},{}",
                t, d.acc[0], d.acc[1], d.acc[2], d.gyr[0], d.gyr[1], d.gyr[2], m0, m1, m2, m3
            );
            ulog::log(s.as_str());
        } else {
            // Line B: PID internals + attitude + thrust
            let pid = pid_rcv.try_get();
            let rsp = ref_rcv.try_get();
            let thr = thr_rcv.try_get().unwrap_or(0.0);
            let att = att_rcv.try_get();

            if let Some(pid) = pid {
                let r = rsp.unwrap_or([0.0; 3]);
                // Euler angles from attitude quaternion
                let (rd, pd, yd) = match att {
                    Some(q) => {
                        let (r, p, y) = q.euler_angles();
                        (r.to_degrees(), p.to_degrees(), y.to_degrees())
                    }
                    None => (0.0, 0.0, 0.0),
                };
                // Line B1: attitude + roll PID + pitch PID
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "B,{},{:.1},{:.1},{:.1},{:.2},{:.3},{:.3},{:.2},{:.3},{:.3},{:.1}",
                    t,
                    rd,
                    pd,
                    yd,
                    r[0],
                    pid[0].p_out,
                    pid[0].i_out,
                    r[1],
                    pid[1].p_out,
                    pid[1].i_out,
                    thr
                );
                ulog::log(s.as_str());
            }
        }
        cycle = 1 - cycle;
    }
}

#[embassy_executor::task]
async fn motor_monitor() -> ! {
    // Reacts immediately to state changes (arming, disarm, timeout).
    // Armed speed is rate-limited to ~0.5 Hz to avoid flooding the log.
    let mut rcv = signals::MOTORS_STATE.receiver();
    let mut last_armed_log = embassy_time::Instant::now();
    loop {
        let state = rcv.changed().await;
        let mut s: heapless::String<64> = heapless::String::new();
        match state {
            MotorsState::Disarmed(reason) => {
                let _ = write!(s, "[mtr] disarmed ({:?})", reason);
                ulog::log(s.as_str());
            }
            MotorsState::Arming => {
                ulog::log("[mtr] arming");
            }
            MotorsState::ArmedIdle => {
                ulog::log("[mtr] armed-idle");
            }
            MotorsState::Armed(sp) => {
                if last_armed_log.elapsed().as_millis() >= 2000 {
                    let _ = write!(s, "[mtr] [{},{},{},{}]", sp[0], sp[1], sp[2], sp[3]);
                    ulog::log(s.as_str());
                    last_armed_log = embassy_time::Instant::now();
                }
            }
        }
    }
}

// ------------------------------------------------------------------
// Auto-kill on flip detection.
//
// Monitors the accelerometer Z axis. If gravity points upward (az < 0)
// for more than FLIP_DURATION_MS, the drone is inverted and motors are
// immediately disarmed. This catches the most dangerous failure mode
// (uncontrolled flip) without requiring any external hardware.
// ------------------------------------------------------------------

/// Master enable for the flip-kill safety feature.
/// Set to false to disable (e.g., for inverted bench testing).
const FLIP_KILL_ENABLED: bool = true;

#[embassy_executor::task]
async fn flip_kill() -> ! {
    if !FLIP_KILL_ENABLED {
        ulog::log("[kill] flip-kill DISABLED");
        loop {
            Timer::after_secs(60).await;
        }
    }

    ulog::log("[kill] flip-kill active");

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();

    // Number of consecutive inverted samples before triggering kill.
    // At 1kHz IMU rate: cnt=10 = 10ms. Filters motor vibration spikes
    // (1-2ms) while catching real flips (50-100ms) within 10ms.
    const FLIP_COUNT_THRESHOLD: u32 = 10;
    // Z acceleration below this threshold means inverted (gravity pointing up).
    const AZ_INVERTED_THRESHOLD: f32 = -3.0;

    let mut inverted_count: u32 = 0;

    loop {
        let d = rcv.changed().await;

        if d.acc[2] < AZ_INVERTED_THRESHOLD {
            inverted_count += 1;
            if inverted_count >= FLIP_COUNT_THRESHOLD {
                COMMAD_ARM_VEHICLE.send(false);
                ulog::log("[kill] FLIP DETECTED -- motors disarmed");

                // Log a few more readings then go quiet.
                for _ in 0..5 {
                    Timer::after_millis(200).await;
                    ulog::log("[kill] motors off (flip-kill)");
                }

                // Stay disarmed forever (requires power cycle to reset).
                loop {
                    Timer::after_secs(60).await;
                }
            }
        } else {
            inverted_count = 0;
        }
    }
}

// ------------------------------------------------------------------
// Gyro-runaway autoabort.
//
// If any axis gyro magnitude exceeds GYRO_RUNAWAY_THRESHOLD for
// GYRO_RUNAWAY_COUNT consecutive samples, the controller is clearly
// diverging and must be stopped before the drone builds enough momentum
// to break its restraint. This is a MUCH earlier trigger than flip_kill,
// which only fires once the drone is already inverted. Scoped to this
// test binary because in a real flight some axes legitimately exceed
// 5 rad/s during aggressive manoeuvres; we're testing sub-liftoff, so
// any gyro activity that big means something has gone wrong.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn gyro_runaway_kill() -> ! {
    ulog::log("[kill] gyro-runaway autoabort active");

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();

    /// rad/s. At sub-hover the physical drone should stay well under this.
    const GYRO_RUNAWAY_THRESHOLD: f32 = 5.0;
    /// 50 samples at 1 kHz IMU rate = 50 ms.
    const GYRO_RUNAWAY_COUNT: u32 = 50;

    let mut count: u32 = 0;

    loop {
        let d = rcv.changed().await;
        let gmax = d.gyr[0].abs().max(d.gyr[1].abs()).max(d.gyr[2].abs());

        if gmax > GYRO_RUNAWAY_THRESHOLD {
            count += 1;
            if count >= GYRO_RUNAWAY_COUNT {
                COMMAD_ARM_VEHICLE.send(false);
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "[kill] GYRO RUNAWAY gx={:.1} gy={:.1} gz={:.1} -- motors disarmed",
                    d.gyr[0], d.gyr[1], d.gyr[2]
                );
                ulog::log(s.as_str());
                for _ in 0..5 {
                    Timer::after_millis(200).await;
                    ulog::log("[kill] motors off (gyro-runaway)");
                }
                loop {
                    Timer::after_secs(60).await;
                }
            }
        } else {
            count = 0;
        }
    }
}

// Note: flow_hold and mtf01_reader_task are intentionally omitted in this
// binary. No optical-flow-driven attitude setpoints -- TRUE_ATTITUDE_Q_SP
// stays at the identity quaternion set in main(), so the rate loop is
// trying to hold a level attitude for the whole test.
