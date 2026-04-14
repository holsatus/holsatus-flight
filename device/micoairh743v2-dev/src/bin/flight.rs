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
    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);

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
    thread_spawner.spawn(mission_sequencer().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());

    ulog::log("[flight] all tasks spawned");

    apply_imu_calibration().await;

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
async fn override_pid_gains() {
    use common::tasks::controller_rate;

    let mut r = controller_rate::params::TABLE.params.write().await;
    // Roll (x) -- reduced from 0.08 after mixer arm-length fix (effective gain ~1.6x)
    r.x.kp = 0.05;
    r.x.ki = 0.10;
    r.x.kd = 0.015;
    // Pitch (y) -- same as roll
    r.y.kp = 0.05;
    r.y.ki = 0.10;
    r.y.kd = 0.015;
    // Yaw (z) -- minimal gains
    r.z.kp = 0.01;
    r.z.ki = 0.01;
    r.z.kd = 0.005;
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

/// Hardcoded IMU bias and axis scale from flat-surface calibration.
/// Re-calibrate if the FC is remounted or the frame changes.
async fn apply_imu_calibration() {
    use common::tasks::imu_reader;
    use micoairh743v2::config::BMI088_ACC_BIAS;

    let mut p = imu_reader::params::TABLE.params.write().await;
    p.cal_acc.bias = BMI088_ACC_BIAS;
    p.cal_acc.scale = [1.0, -1.0, -1.0];
    p.cal_gyr.scale = [1.0, -1.0, -1.0];
    drop(p);

    imu_reader::CHANNEL[0]
        .sender()
        .send(imu_reader::Message::ReloadParams)
        .await;

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s, "[cal] bias=[{:.3},{:.3},{:.3}]",
        BMI088_ACC_BIAS[0], BMI088_ACC_BIAS[1], BMI088_ACC_BIAS[2]
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
        // No SD -- fall back to UART-only logging.
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
        Err(_) => loop {
            let msg = ulog::CHANNEL.receive().await;
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
        },
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
        Err(_) => loop {
            let msg = ulog::CHANNEL.receive().await;
            if let Some(ref mut u) = uart {
                u.write(msg.as_bytes()).await.ok();
                u.write(b"\r\n").await.ok();
            }
        },
    };

    let mut flush_counter: u16 = 0;
    loop {
        let msg = ulog::CHANNEL.receive().await;

        // Write to UART (if connected).
        if let Some(ref mut u) = uart {
            u.write(msg.as_bytes()).await.ok();
            u.write(b"\r\n").await.ok();
        }

        // Write to SD file.
        file.write_all(msg.as_bytes()).await.ok();
        file.write_all(b"\r\n").await.ok();

        // Flush every 5 messages (~50ms at 100 Hz).
        // Aggressive flush ensures crash data is captured.
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
    // String-suspended slow-ramp hover test.
    //
    // The drone hangs from a string. Thrust ramps from zero to hover over
    // 30 seconds, holds for 30 seconds, then ramps down over 15 seconds.
    // The slow ramp avoids the sudden ground-effect transient that caused
    // previous crashes. Pull LiPo at any sign of instability.
    //
    // Procedure:
    //   1. Suspend drone from a string (loop around the frame)
    //   2. Connect LiPo -- motors arm after ~5 seconds
    //   3. 30s ramp: thrust increases very gradually
    //   4. 30s hover at 0.15m setpoint
    //   5. 15s ramp down, then disarm

    ulog::log("[mission] waiting 3s for sensors...");
    Timer::after_secs(3).await;

    ulog::log("[mission] arming motors");
    COMMAD_ARM_VEHICLE.send(true);
    Timer::after_secs(3).await;

    const TARGET_ALT: f32 = 0.15;
    const RAMP_UP_MS: u64 = 30_000;
    const HOVER_S: u64 = 30;
    const RAMP_DOWN_MS: u64 = 15_000;

    ulog::log("[mission] ramp 0->0.15m (30s)");
    let ramp_start = embassy_time::Instant::now();
    loop {
        let elapsed = ramp_start.elapsed().as_millis();
        if elapsed >= RAMP_UP_MS {
            break;
        }
        let sp = TARGET_ALT * (elapsed as f32 / RAMP_UP_MS as f32);
        ALTITUDE_SETPOINT.signal(sp);
        Timer::after_millis(100).await;
    }
    ALTITUDE_SETPOINT.signal(TARGET_ALT);

    ulog::log("[mission] hover 0.15m (30s)");
    Timer::after_secs(HOVER_S).await;

    ulog::log("[mission] descend (15s)");
    let desc_start = embassy_time::Instant::now();
    loop {
        let elapsed = desc_start.elapsed().as_millis();
        if elapsed >= RAMP_DOWN_MS {
            break;
        }
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
// Flow hold: velocity damping using MTF-01 optical flow.
//
// Reads horizontal velocity from FLOW_VEL_MS and tilts the attitude
// setpoint (TRUE_ATTITUDE_Q_SP) to counteract drift. When the drone
// drifts left, the flow sensor detects leftward ground motion and the
// controller tilts right to brake.
//
// This is velocity hold (zero velocity), not position hold. The drone
// won't return to a fixed point but will resist being pushed around.
// ------------------------------------------------------------------

#[embassy_executor::task]
async fn flow_hold() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();

    let mut snd_att = signals::TRUE_ATTITUDE_Q_SP.sender();

    // Proportional gain: how much tilt (rad) per unit velocity (m/s).
    // Start conservative. Typical range: 0.05-0.3.
    // Too high -> oscillates laterally. Too low -> drift isn't corrected.
    const KP_FLOW: f32 = 0.10;

    // Max tilt angle in radians (~10 degrees). Prevents the flow controller
    // from commanding extreme angles that could flip the drone.
    const MAX_TILT_RAD: f32 = 0.17;

    ulog::log("[flow] velocity damping started");

    let mut log_div: u8 = 0;

    loop {
        let [vx, vy] = rcv.changed().await;

        let pitch_tilt = (KP_FLOW * vx).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll_tilt = (-KP_FLOW * vy).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        let q = UnitQuaternion::from_euler_angles(roll_tilt, pitch_tilt, 0.0);
        snd_att.send(q);

        // Log at ~10 Hz (flow arrives at ~50 Hz, log every 5th).
        log_div = log_div.wrapping_add(1);
        if log_div >= 5 {
            log_div = 0;
            // Read latest flow quality from the mtf01_reader via a shared atomic.
            let fq = FLOW_QUALITY.load(Ordering::Relaxed);
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[flow] q={} vx={:.3} vy={:.3} r={:.2} p={:.2}",
                fq, vx, vy, roll_tilt, pitch_tilt
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
