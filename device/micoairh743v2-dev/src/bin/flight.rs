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

use core::sync::atomic::{AtomicU8, Ordering};

use defmt_rtt as _;
use panic_probe as _;

/// Latest optical flow quality, shared between mtf01_reader and flow_hold for logging.
static FLOW_QUALITY: AtomicU8 = AtomicU8::new(0);

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
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Timer;
use micoairh743v2::alt_hold::ALTITUDE_SETPOINT;
use micoairh743v2::log as ulog;
use micoairh743v2::mtf01;
use micoairh743v2::resources::{
    self, BatteryResources, Mtf01Resources, SdmmcLogResources, UartLogResources,
};
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

    let battery_mv = read_battery_mv(r.battery);

    // ------------------------------------------------------------------
    // Thread-priority tasks
    // ------------------------------------------------------------------
    thread_spawner.spawn(resources::alt_hold_task(r.baro, battery_mv).unwrap());
    thread_spawner.spawn(mtf01_reader_task(r.mtf01).unwrap());
    thread_spawner.spawn(flow_hold().unwrap());
    thread_spawner.spawn(flip_kill().unwrap());
    thread_spawner.spawn(gyro_runaway_kill().unwrap());
    thread_spawner.spawn(micoairh743v2::rc_kill::rc_kill_task(r.rc).unwrap());
    thread_spawner.spawn(mission_sequencer().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());

    ulog::log("[flight] all tasks spawned");

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
        // 15 s timeout: freshly formatted cards can take 5-10 s on first
        // mount (FS init, 4-bit bus negotiation). A warm card usually
        // resolves in under 2 s.
        let wait_start = embassy_time::Instant::now();
        while ulog::SD_MOUNTED.load(Ordering::Relaxed) == 255
            && wait_start.elapsed().as_millis() < 15_000
        {
            Timer::after_millis(100).await;
        }
    }
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[flight] SD ABORT -- motors will not arm (3x blue + 1x red forever)");
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
        let _ = write!(s, "[flight] hb dshot={}Hz", rate);
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
    let _ = write!(s, "[flight] rev=0x{:04X} timeout=500ms", MOTOR_REVERSE_FLAGS.bits());
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

    ulog::log("[flight] PID gains overridden");
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

    const WINDOW_N: usize = 50;      // 500ms at 100 Hz sample rate
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
        if filled < WINDOW_N { filled += 1; }

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
                s, "[ahrs] WARNING: not settled after {}ms, continuing anyway", MAX_WAIT_MS
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
        s, "[cal] acc=[{:.3},{:.3},{:.3}]",
        acc_bias[0], acc_bias[1], acc_bias[2]
    );
    ulog::log(s.as_str());

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s, "[cal] gyr=[{:.4},{:.4},{:.4}]",
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

    bind_interrupts!(struct UartIrqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    });

    let mut uart = UartTx::new(r.usart, r.tx, r.dma, UartIrqs, UartConfig::default()).ok();

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
    let _ = FmtWrite::write_fmt(&mut s, format_args!("[sd] mounted -> {}/000001.LOG", dir_name.as_str()));
    ulog::log(s.as_str());

    let mut flush_counter: u16 = 0;
    loop {
        // Priority: drain CRITICAL_CHANNEL (kill/restart announcements)
        // before ever touching the regular CHANNEL. Uses try_receive in a
        // tight loop so back-to-back critical messages are all flushed
        // before we service telemetry.
        while let Ok(msg) = ulog::CRITICAL_CHANNEL.try_receive() {
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
            file.write_all(msg.as_bytes()).await.ok();
            file.write_all(b"\r\n").await.ok();
            file.flush().await.ok();
        }

        // Now await either a new critical message OR a regular one.
        use embassy_futures::select::{select, Either};
        let msg = match select(
            ulog::CRITICAL_CHANNEL.receive(),
            ulog::CHANNEL.receive(),
        )
        .await
        {
            Either::First(m) => {
                // Critical -- always to UART + SD, always flushed.
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

        // Regular message. Suppress high-rate CSV telemetry on UART.
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

#[embassy_executor::task]
async fn mission_sequencer() -> ! {
    // Ground-start hover test with slow thrust ramp.
    //
    // Validated in holsatus-sim SITL with realistic MTF-01 noise.
    //
    // Phase 0: SLOW THRUST RAMP (10s). Thrust increases linearly from 0
    //   to BASE_THRUST. Liftoff occurs around second 5-6 when thrust
    //   exceeds weight. This gives ~5 seconds of "motors spinning, drone
    //   on ground" to verify stability and pull the plug if needed.
    // Phase 1: altitude setpoint ramp 0 -> 1.0m (10s)
    // Phase 2: hover at 1.0m (10s)
    // Phase 3: altitude setpoint ramp 1.0m -> 0 (10s)
    // Phase 4: disarm

    ulog::log("[mission] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    // SD card must be mounted before we consider arming. main() handles the
    // LED pattern; here we just refuse to proceed so no motor ever spins
    // without telemetry being captured.
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[mission] ABORT: no SD card, not arming");
        loop { Timer::after_secs(60).await; }
    }

    // Verify Madgwick has converged to a stable attitude before arming.
    // If this logs WARNING, the drone was vibrating/moving during calibration
    // and the attitude estimate may be unreliable -- abort and retry.
    wait_for_ahrs_ready().await;

    // Wait for RC link: the kill switch must be functional before any motor
    // spins. rc_kill_task sets RC_LINK_READY once it has parsed its first
    // valid CRSF packet and recorded baseline SE/SF positions.
    {
        ulog::log("[mission] waiting for RC link (30 s timeout)...");
        let start = embassy_time::Instant::now();
        const RC_WAIT_MS: u64 = 30_000;
        while !micoairh743v2::rc_kill::RC_LINK_READY
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            if start.elapsed().as_millis() >= RC_WAIT_MS {
                ulog::log("[mission] ABORT: no RC link after 30 s -- motors will NOT arm");
                loop { Timer::after_secs(60).await; }
            }
            Timer::after_millis(100).await;
        }
        ulog::log("[mission] RC link established");
    }

    ulog::log("[mission] arming motors");
    COMMAD_ARM_VEHICLE.send(true);
    Timer::after_secs(3).await;

    // Phase 0: slow thrust ramp. Bypasses alt_hold by writing directly
    // to TRUE_Z_THRUST_SP. alt_hold also writes this signal, but it
    // starts at err=0/sp=0 so its output is BASE_THRUST (hover).
    // During the ramp we override it at 50 Hz (faster than alt_hold's 10 Hz).
    const BASE_THRUST: f32 = 4.50;
    const RAMP_S: u64 = 10;
    ulog::log("[mission] P0:thrust ramp (10s, liftoff ~5s)");
    let ramp_start = embassy_time::Instant::now();
    loop {
        let elapsed = ramp_start.elapsed().as_millis();
        if elapsed >= RAMP_S * 1000 { break; }
        let frac = elapsed as f32 / (RAMP_S * 1000) as f32;
        signals::TRUE_Z_THRUST_SP.send(BASE_THRUST * frac);
        Timer::after_millis(20).await;
    }

    // Hand off to alt_hold (which runs continuously and will now take over
    // TRUE_Z_THRUST_SP at its own rate).
    const TARGET_ALT: f32 = 1.0;
    const RAMP_UP_MS: u64 = 10_000;
    const HOVER_S: u64 = 10;
    const RAMP_DOWN_MS: u64 = 10_000;

    ulog::log("[mission] P1:climb 0->1.0m (10s)");
    let ramp_start = embassy_time::Instant::now();
    loop {
        let elapsed = ramp_start.elapsed().as_millis();
        if elapsed >= RAMP_UP_MS { break; }
        let sp = TARGET_ALT * (elapsed as f32 / RAMP_UP_MS as f32);
        ALTITUDE_SETPOINT.signal(sp);
        Timer::after_millis(100).await;
    }
    ALTITUDE_SETPOINT.signal(TARGET_ALT);

    ulog::log("[mission] P2:hover 1.0m (10s)");
    Timer::after_secs(HOVER_S).await;

    ulog::log("[mission] P3:descend (10s)");
    let desc_start = embassy_time::Instant::now();
    loop {
        let elapsed = desc_start.elapsed().as_millis();
        if elapsed >= RAMP_DOWN_MS { break; }
        let sp = TARGET_ALT * (1.0 - elapsed as f32 / RAMP_DOWN_MS as f32);
        ALTITUDE_SETPOINT.signal(sp);
        Timer::after_millis(100).await;
    }
    ALTITUDE_SETPOINT.signal(0.0);

    Timer::after_secs(3).await;

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
// GYRO_RUNAWAY_COUNT consecutive samples, the controller is diverging
// and is stopped before the drone can break restraint / tumble. The
// threshold of 5 rad/s (~290 deg/s) sustained for 50 ms is conservative
// for the gentle hover regime this binary targets; raise it before
// any aggressive acro flight.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn gyro_runaway_kill() -> ! {
    ulog::log("[kill] gyro-runaway autoabort active");

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();

    const GYRO_RUNAWAY_THRESHOLD: f32 = 5.0;
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
                loop { Timer::after_secs(60).await; }
            }
        } else {
            count = 0;
        }
    }
}

// ------------------------------------------------------------------
// Flow-based position hold using MTF-01 optical flow.
//
// Integrates flow velocity into a dead-reckoned position estimate,
// then uses PD control (position + velocity) to command attitude tilts.
// This provides actual station-keeping, not just velocity damping.
//
// The position estimate drifts slowly (no absolute reference), but for
// short flights (<60s) the drift is small enough for ~10cm hold.
// Validated in holsatus-sim SITL with realistic MTF-01 noise model.
//
// Key difference from the previous velocity-only flow_hold: velocity
// damping gave ~2.3m std dev in sim. Position integration + PD gives
// ~0.05m std dev.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn flow_hold() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();

    let mut snd_att = signals::TRUE_ATTITUDE_Q_SP.sender();

    // PD position hold gains (validated in sim).
    const KP_POS: f32 = 0.30;  // rad/m -- tilt per metre of estimated position error
    const KD_VEL: f32 = 0.25;  // rad/(m/s) -- tilt per m/s of measured velocity
    const MAX_TILT_RAD: f32 = 0.17; // ~10 degrees

    // Integrated position estimate (body frame, starts at origin).
    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;

    // Wait until motors are armed before publishing attitude setpoints.
    // Without this gate, the task starts integrating spurious MTF-01
    // velocities from boot, hits the 0.5 m position clamp, and commands
    // an 8.6 degree tilt setpoint that the angle controller then tries
    // to track -- producing the 4-6 rad/s rate setpoints visible in
    // D000019 while the drone was held stationary. The sim version in
    // std-test-device/src/lib.rs::h743v2_flow_hold uses the same gate.
    {
        use common::types::actuators::MotorsState;
        let mut mtr_rcv = common::signals::MOTORS_STATE.receiver();
        loop {
            match mtr_rcv.changed().await {
                MotorsState::ArmedIdle | MotorsState::Armed(_) => break,
                _ => {}
            }
        }
    }

    ulog::log("[flow] motors armed, position hold active");

    let mut log_div: u8 = 0;

    loop {
        let [vx, vy] = rcv.changed().await;

        // Safeguard 1: spike rejection. The MTF-01 occasionally produces
        // large spurious readings (seen vy=-0.878 in D000302). Skip these
        // to prevent the position estimate from jumping.
        const MAX_FLOW_VEL: f32 = 1.0;
        if vx.abs() > MAX_FLOW_VEL || vy.abs() > MAX_FLOW_VEL {
            continue;
        }

        // Safeguard 2: flow quality gate. mtf01_reader already gates at
        // quality > 30, but be stricter here for position integration.
        let fq = FLOW_QUALITY.load(Ordering::Relaxed);
        if fq < 50 {
            continue;
        }

        // Integrate velocity to estimate position (dead reckoning).
        const DT: f32 = 0.02;
        est_x += vx * DT;
        est_y += vy * DT;

        // Safeguard 3: position estimate clamp. If the dead-reckoned
        // position exceeds 0.5m, freeze position term and fall back to
        // velocity-only damping. Prevents chasing a wrong estimate into
        // a wall.
        const POS_CLAMP: f32 = 0.5;
        let pos_ok = est_x.abs() < POS_CLAMP && est_y.abs() < POS_CLAMP;
        est_x = est_x.clamp(-POS_CLAMP, POS_CLAMP);
        est_y = est_y.clamp(-POS_CLAMP, POS_CLAMP);

        let kp = if pos_ok { KP_POS } else { 0.0 };

        // PD: pitch corrects forward/back, roll corrects left/right.
        let pitch_cmd = ( (kp * est_x + KD_VEL * vx)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll_cmd  = (-(kp * est_y + KD_VEL * vy)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        let q = UnitQuaternion::from_euler_angles(roll_cmd, pitch_cmd, 0.0);
        snd_att.send(q);

        // Log at ~10 Hz (flow arrives at ~50 Hz, log every 5th).
        log_div = log_div.wrapping_add(1);
        if log_div >= 5 {
            log_div = 0;
            let mode = if pos_ok { 'P' } else { 'V' };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[flow] {}q={} ex={:.2} ey={:.2} vx={:.2} vy={:.2} p={:.2} r={:.2}",
                mode, fq, est_x, est_y, vx, vy, pitch_cmd, roll_cmd
            );
            ulog::log(s.as_str());
        }
    }
}

// ------------------------------------------------------------------
// MTF-01 optical flow + lidar reader.
// Reads MSP v2 frames at ~100 Hz and publishes lidar distance to
// LIDAR_ALT_M for the altitude controller.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn mtf01_reader_task(r: Mtf01Resources) -> ! {
    use embassy_stm32::usart::{Config as UartConfig, UartRx};

    bind_interrupts!(struct Mtf01Irqs {
        DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
        UART4        => embassy_stm32::usart::InterruptHandler<peripherals::UART4>;
    });

    let mut cfg = UartConfig::default();
    cfg.baudrate = 115_200;
    let mut uart = UartRx::new(r.usart, r.rx, r.dma, Mtf01Irqs, cfg).unwrap();

    ulog::log("[mtf01] UART4 RX started");

    let snd_lidar = micoairh743v2::alt_hold::LIDAR_ALT_M.sender();
    let snd_flow = micoairh743v2::alt_hold::FLOW_VEL_MS.sender();
    let mut last_height_m: f32 = 0.0;

    // Read buffers (reused every frame).
    let mut b = [0u8; 1];
    let mut hdr = [0u8; 6];
    let mut pbuf = [0u8; mtf01::MAX_PAYLOAD + 1];

    loop {
        // Sync to MSP v2 preamble: $X
        loop {
            uart.read(&mut b).await.ok();
            if b[0] != b'$' {
                continue;
            }
            uart.read(&mut b).await.ok();
            if b[0] == b'X' {
                break;
            }
        }

        // Read header: dir flags fn_lo fn_hi size_lo size_hi
        uart.read(&mut hdr).await.ok();
        let size = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if size > mtf01::MAX_PAYLOAD {
            continue;
        }

        // Read payload + CRC
        uart.read(&mut pbuf[..size + 1]).await.ok();

        match mtf01::parse(hdr, &pbuf[..size + 1]) {
            Some(mtf01::Frame::Lidar(l)) => {
                if l.quality > 0 && l.distance_mm >= 0 {
                    let h = l.distance_mm as f32 / 1000.0;
                    snd_lidar.send(h);
                    last_height_m = h;
                }
            }
            Some(mtf01::Frame::Flow(f)) => {
                FLOW_QUALITY.store(f.quality, Ordering::Relaxed);
                if f.quality > 30 && last_height_m > 0.05 {
                    // Convert pixel counts to velocity via angular rate * height.
                    // Axis mapping (empirically determined from flow_test):
                    //   drone forward = +motion_y
                    //   drone right   = -motion_x
                    // Scale: raw ±5 counts at 8cm height ≈ 0.1 m/s hand movement.
                    //   FLOW_SCALE = 0.1 / (5 * 0.08) = 0.25
                    const FLOW_SCALE: f32 = 0.25;
                    let vx = f.motion_y as f32 * FLOW_SCALE * last_height_m;
                    let vy = -(f.motion_x as f32) * FLOW_SCALE * last_height_m;
                    snd_flow.send([vx, vy]);
                }
            }
            None => {}
        }
    }
}
