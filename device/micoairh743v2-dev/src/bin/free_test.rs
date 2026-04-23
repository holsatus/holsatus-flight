//! MicoAir H743 -- thrust staircase test binary.
//!
//! Built on flight.rs task structure verbatim (known working reference).
//! The only difference from flight.rs is the mission body: instead of the
//! altitude-hold climb/hover/descend sequence, this binary executes a
//! 6-step thrust staircase for motor and controller characterisation.
//!
//! Mission:
//!   P0: 1 s open-loop thrust ramp to BASE_THRUST (pushes through stick-slip)
//!   P1: 3 s closed-loop climb 0 -> 1.0 m (alt_hold takes over TRUE_Z_THRUST_SP)
//!   P2: 1 s hover at 1.0 m (shortened from 15 s to avoid drift into walls
//!       before we solve flow-induced lateral drift)
//!   P3: 1 s fast descent 1.0 -> 0.25 m, then 1 s slow approach 0.25 -> 0.05 m,
//!       with lidar-based ground-detect disarm (h<0.15m for 300 ms) and a
//!       5 s safety timeout. flow_hold silent below 25 cm.
//!   Total arm-to-disarm: ~22 s
//!   rate-PID gains at flight.rs boot defaults
//!
//! Range shifted down two stops in D000064+ after the landing frame was
//! changed from Kapla blocks to corks + chopstick criss-cross (lighter,
//! wider stance). D000063 showed the drone already yaw-spinning at 120
//! percent, so hover thrust on the new build is below the old starting
//! point. 90-120 percent now covers the pre-liftoff to lift-transition
//! range where the interesting behaviour happens.
//!
//! IMPORTANT: the drone WILL lift off during this test, possibly within
//! the first step. Be ready on the TX15 SE kill switch at all times.
//!
//! Gyro-runaway autoabort at 5 rad/s.
//!
//! SAFETY:
//!   - Props on, landing cage (Kapla sticks) installed, textured floor
//!     (newspaper or similar -- bare wood starves flow_hold per D000033)
//!   - 1.5-2 m clear on all sides
//!   - TX15 SE kill switch tested on the bench BEFORE arming flight
//!     (motors must go silent the instant SE flips)
//!   - Firmware backups: flip_kill (az < -3 m/s^2 for 10 ms),
//!     gyro_runaway_kill (any axis > 5 rad/s for 50 ms)
//!   - Battery is belly-mounted inside the landing cage; it is NOT
//!     reachable in flight, so SE on the TX15 is the only human kill

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use defmt_rtt as _;
use panic_probe as _;

/// Latest optical flow quality, shared between mtf01_reader and flow_hold for logging.
static FLOW_QUALITY: AtomicU8 = AtomicU8::new(0);

/// Unclamped flow-integrated body-frame displacement since the first flow
/// sample (mm). Updated by `flow_position_logger`, read at disarm by the
/// mission sequencer for landing-distance estimation. Separate from
/// `flow_hold`'s internal est_x/est_y because flow_hold clamps at +/-0.5 m,
/// which would saturate for the ~2 m drifts we see in practice and lose
/// information. These atomics are pure telemetry -- nothing in the control
/// loop reads them.
static FLOW_EST_X_MM: AtomicI32 = AtomicI32::new(0);
static FLOW_EST_Y_MM: AtomicI32 = AtomicI32::new(0);

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
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let r = resources::split(p);

    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);
    let mut led_blue = Output::new(r.leds.blue, Level::Low, Speed::Low);
    let mut led_red = Output::new(r.leds.red, Level::Low, Speed::Low);

    thread_spawner.spawn(uart_writer_task(r.uart_log, r.sdmmc).unwrap());

    ulog::log("[free] board init ok");
    // Git provenance line: every flight log now starts with the short SHA
    // that produced the binary (plus "-dirty" if the tree was not clean at
    // build time). Combined with the Makefile's `git-clean` pre-flash gate,
    // this gives end-to-end version tracing from log back to a commit.
    ulog::log(concat!("[free] git=", env!("GIT_SHA")));

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
    signals::TRUE_ATTITUDE_Q_SP.send(UnitQuaternion::identity());

    thread_spawner.spawn(param_storage_task().unwrap());

    override_motor_params().await;
    override_pid_gains().await;
    override_imu_rot().await;

    Timer::after_millis(10).await;

    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    let level_1_spawner = interrupt_executor!(FDCAN1_IT1, P11);

    ulog::log("[free] executors started");

    Timer::after_millis(1).await;

    level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());
    level_0_spawner.spawn(resources::motor_governor_task(r.motors, DshotConfig::Dshot300).unwrap());
    level_0_spawner.spawn(controller_rate::main().unwrap());

    // Calibrate IMU BEFORE spawning att_estimator. Otherwise Madgwick
    // integrates 2-3 s of uncalibrated (biased) gyro data into the attitude
    // estimate before ReloadParams takes effect. At beta=0.03 the filter
    // takes ~30 s to un-wind that error via accel correction, and we arm in
    // ~10 s -- so the drone arms with 5-10 deg of fake tilt baked into the
    // estimate (see D000063: [cal] gyr bias ~3 deg/s, [ahrs] ready reported
    // p=8 deg when the drone was physically level). The angle controller then
    // commands 2+ rad/s "correction" on arming, tipping the real drone.
    micoairh743v2::imu_cal::apply().await;

    level_1_spawner.spawn(att_estimator::main().unwrap());
    level_1_spawner.spawn(ahrs_to_eskf_bridge().unwrap());
    level_1_spawner.spawn(controller_angle::main().unwrap());
    level_1_spawner.spawn(angle_to_rate_bridge().unwrap());

    let battery_mv = read_battery_mv(r.battery);

    thread_spawner.spawn(resources::alt_hold_task(r.baro, battery_mv).unwrap());
    thread_spawner.spawn(mtf01_reader_task(r.mtf01).unwrap());
    // flow_hold re-enabled 2026-04-22 in velocity-damping-only mode (KP_POS=0)
    // to counter the steady left-drift seen in D000116 without triggering the
    // D000049 positive-feedback failure at borderline liftoff. The activation
    // threshold in mtf01_reader_task was also raised from 5 cm to 10 cm so
    // flow only engages once the drone is solidly airborne, not in the
    // transition regime. See also: flow_hold::KP_POS = 0.0 below.
    thread_spawner.spawn(flow_hold().unwrap());
    thread_spawner.spawn(flip_kill().unwrap());
    thread_spawner.spawn(gyro_runaway_kill().unwrap());
    thread_spawner.spawn(micoairh743v2::rc_kill::rc_kill_task(r.rc).unwrap());
    thread_spawner.spawn(staircase_mission().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());
    thread_spawner.spawn(flow_position_logger().unwrap());
    thread_spawner.spawn(mag_yaw_logger().unwrap());

    ulog::log("[free] all tasks spawned");

    {
        let wait_start = embassy_time::Instant::now();
        while ulog::SD_MOUNTED.load(Ordering::Relaxed) == 255
            && wait_start.elapsed().as_millis() < 15_000
        {
            Timer::after_millis(100).await;
        }
    }
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[free] SD ABORT -- motors will not arm (3x blue + 1x red forever)");
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

    let mut prev_tx =
        micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
    loop {
        led_green.toggle();
        Timer::after_secs(2).await;
        let cur_tx =
            micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
        let rate = (cur_tx - prev_tx) / 2;
        prev_tx = cur_tx;
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = write!(s, "[free] hb dshot={}Hz", rate);
        ulog::log(s.as_str());
    }
}

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

async fn override_motor_params() {
    use common::tasks::motor_governor::params;
    use micoairh743v2::config::MOTOR_REVERSE_FLAGS;

    let mut p = params::TABLE.params.write().await;
    p.rev = MOTOR_REVERSE_FLAGS;
    p.timeout_ms = 500;
    drop(p);

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(s, "[free] rev=0x{:04X} timeout=500ms", MOTOR_REVERSE_FLAGS.bits());
    ulog::log(s.as_str());
}

async fn override_pid_gains() {
    use common::tasks::controller_rate;

    let mut r = controller_rate::params::TABLE.params.write().await;
    r.x.kp = 0.05;
    r.x.ki = 0.0;
    r.x.kd = 0.002;
    r.y.kp = 0.05;
    r.y.ki = 0.0;
    r.y.kd = 0.002;
    // Yaw Kp bumped 0.04 -> 0.07 after D000106 showed ~0.26 rad/s yaw drift
    // at airborne step 3 due to unmodelled CW/CCW drag asymmetry. Ki stays 0
    // because the rate controller's integrators are gated off on this board
    // (in_flight_estimator is not spawned, ATTITUDE_INT_EN never fires) so
    // setting ki would be a no-op -- see project_rate_ki_gated_off memo.
    r.z.kp = 0.07;
    r.z.ki = 0.0;
    r.z.kd = 0.001;
    drop(r);

    ulog::log("[free] PID gains overridden");
}

/// Configure the BMI088 chip-to-drone-body rotation matrix so that
/// RAW_MULTI_IMU_DATA (and downstream CAL_MULTI_IMU_DATA, ESKF, Madgwick,
/// rate/angle controllers) operate in the drone's arrow-forward NED frame
/// rather than the chip's native frame. Without this override, the default
/// rotation is identity -- which is wrong because the BMI088 on the MicoAir
/// H743v2 is physically mounted rotated 180 deg around the (1,-1,0) diagonal
/// relative to the drone's arrow. See `BMI088_CHIP_TO_DRONE_ROT` in config.rs
/// for the derivation and the empirical tilt-test evidence (2026-04-21).
async fn override_imu_rot() {
    use common::tasks::imu_reader::params;
    use common::utils::rot_matrix::Rotation;
    use micoairh743v2::config::BMI088_CHIP_TO_DRONE_ROT;

    let mut p = params::TABLE.params.write().await;
    p.rot = Rotation::Custom(BMI088_CHIP_TO_DRONE_ROT);
    drop(p);

    ulog::log("[free] IMU rotation set (chip -> drone NED)");
}

async fn wait_for_ahrs_ready() {
    use signals::AHRS_ATTITUDE_Q;

    const WINDOW_N: usize = 50;
    const STABLE_THRESHOLD_DEG: f32 = 0.3;
    const LEVEL_THRESHOLD_DEG: f32 = 3.0;
    const MAX_WAIT_MS: u64 = 10_000;

    let mut rcv = AHRS_ATTITUDE_Q.receiver();
    let mut roll_buf = [0.0_f32; WINDOW_N];
    let mut pitch_buf = [0.0_f32; WINDOW_N];
    let mut idx: usize = 0;
    let mut filled = 0usize;

    ulog::log("[ahrs] waiting for attitude to settle...");
    let start = embassy_time::Instant::now();

    loop {
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
                // Stable but tilted: abort so the user can level the drone.
                // Arming from a non-zero attitude reference causes the drone
                // to accelerate horizontally at liftoff instead of climbing
                // straight up (see D000065: p=-4.6 deg, drone drifted sideways).
                if r_mean.abs() > LEVEL_THRESHOLD_DEG || p_mean.abs() > LEVEL_THRESHOLD_DEG {
                    let mut s: heapless::String<96> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[ahrs] NOT LEVEL: r={:.2} p={:.2} > {:.1} deg -- ABORT, level the drone",
                        r_mean, p_mean, LEVEL_THRESHOLD_DEG
                    );
                    ulog::log(s.as_str());
                    loop { Timer::after_secs(60).await; }
                }
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

    ulog::SD_MOUNTED.store(1, Ordering::Relaxed);
    let mut s: heapless::String<32> = heapless::String::new();
    let _ = FmtWrite::write_fmt(&mut s, format_args!("[sd] mounted -> {}/000001.LOG", dir_name.as_str()));
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
        let msg = match select(
            ulog::CRITICAL_CHANNEL.receive(),
            ulog::CHANNEL.receive(),
        )
        .await
        {
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

/// Thrust staircase mission. Identical boilerplate to flight.rs's
/// mission_sequencer through the arm + 3s wait; replaces the single 10s
/// thrust ramp and altitude-hold climb/hover/descend with a 6-step
/// staircase over ~78 s.
#[embassy_executor::task]
async fn staircase_mission() -> ! {
    ulog::log("[mission] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[mission] ABORT: no SD card, not arming");
        loop { Timer::after_secs(60).await; }
    }

    wait_for_ahrs_ready().await;

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

    // Closed-loop altitude-hold mission. Replaces the open-loop thrust
    // staircase used through D000128. The staircase was always going to
    // bounce in and out of ground effect: open-loop thrust at a fixed
    // multiple of the firmware's hover-thrust estimate has no authority to
    // compensate for (a) pack-voltage sag, (b) ground-effect lift boost, or
    // (c) thrust-to-weight-estimate mismatch. Altitude-hold closes the loop
    // on lidar altitude and commands whatever thrust is needed to reach and
    // hold a target height, so we don't have to guess.
    //
    // Sequence:
    //   P0: 1 s open-loop ramp to BASE_THRUST (4.5). Gets through the
    //       stick-slip ground-contact zone fast before alt_hold takes over.
    //   P1: 3 s ramp of ALTITUDE_SETPOINT from 0 -> 1.0 m. alt_hold's PID
    //       grabs TRUE_Z_THRUST_SP at its 10 Hz rate once we start signalling
    //       setpoints. 3 s is fast enough to clear ground effect quickly,
    //       slow enough to avoid a climb-rate overshoot.
    //   P2: 15 s hover at 1.0 m. Long enough for flow damping to reach
    //       steady state and for any residual drift to be observable.
    //   P3: 3 s ramp setpoint 1.0 -> 0.0 m. Gentle descent.
    //   P4: setpoint = 0, wait 2 s, disarm.
    // Keep in sync with alt_hold::BASE_THRUST. Lowered 8.50 -> 8.00 after
    // D000018 over-climbed at ~1-2 m/s (baseline thrust 7.8 post-v_comp at
    // BASE=8.5 was ~10% above the drone's actual out-of-ground-effect
    // hover of ~7.0-7.3). See alt_hold.rs for the full calibration chain.
    const BASE_THRUST: f32 = 8.00;
    const TARGET_ALT: f32 = 1.0;
    const P0_RAMP_MS: u64 = 1_000;
    const P1_RAMP_MS: u64 = 3_000;
    // Hover shortened 15s -> 1s after D000028: with flow-induced drift
    // still unresolved, a 15 s hover window gives the drone time to
    // wander into a wall before P3 descent begins. 1 s is enough to
    // confirm altitude hold reached setpoint before triggering landing.
    // Extend again once drift is under control.
    const P2_HOVER_S: u64 = 1;

    ulog::log("[mission] P0: thrust ramp 0 -> base (1s, open-loop)");
    let ramp_start = embassy_time::Instant::now();
    loop {
        let elapsed = ramp_start.elapsed().as_millis() as u64;
        if elapsed >= P0_RAMP_MS { break; }
        let f = elapsed as f32 / P0_RAMP_MS as f32;
        signals::TRUE_Z_THRUST_SP.send(BASE_THRUST * f);
        Timer::after_millis(20).await;
    }

    ulog::log("[mission] P1: climb 0 -> 1.0m (3s, alt_hold owns thrust)");
    let climb_start = embassy_time::Instant::now();
    loop {
        let elapsed = climb_start.elapsed().as_millis() as u64;
        if elapsed >= P1_RAMP_MS { break; }
        let sp = TARGET_ALT * (elapsed as f32 / P1_RAMP_MS as f32);
        ALTITUDE_SETPOINT.signal(sp);
        Timer::after_millis(100).await;
    }
    ALTITUDE_SETPOINT.signal(TARGET_ALT);

    ulog::log("[mission] P2: hover 1.0m (15s)");
    Timer::after_secs(P2_HOVER_S).await;

    // P3 redesigned after D000026 flipped the drone at ~20 cm on descent:
    //
    //   - Fast phase: setpoint 1.0 -> 0.25 m over 1.0 s (~0.75 m/s). Brings
    //     the drone down quickly from cruise altitude to flare-start height.
    //   - Slow phase: setpoint 0.25 -> 0.05 m over 1.0 s (~0.20 m/s). Gentle
    //     approach that trades descent speed for stability near the ground.
    //   - Ground detection: any time lidar reads below 0.15 m continuously
    //     for 300 ms, break out of the loop and disarm. This guards against
    //     the drone sitting at the borderline altitude where flow_hold and
    //     ground effect conspire to flip it.
    //
    // mtf01_reader_task's flow-activation threshold was also raised from
    // 10 cm to 25 cm in the same change, so flow_hold goes silent before
    // the slow phase even begins -- no tilt commands during the critical
    // last metre.
    ulog::log("[mission] P3: descend 1.0 -> 0.05m with ground-detect disarm");
    const P3A_RAMP_MS: u64 = 1_000;
    const P3A_FINAL: f32 = 0.25;
    const P3B_RAMP_MS: u64 = 1_000;
    const P3B_FINAL: f32 = 0.05;
    const GROUND_DETECT_M: f32 = 0.15;
    const GROUND_HOLD_MS: u64 = 300;
    const DESCENT_TIMEOUT_MS: u64 = 5_000;

    let mut lidar_rcv = micoairh743v2::alt_hold::LIDAR_ALT_M.receiver().unwrap();
    let mut ground_since: Option<embassy_time::Instant> = None;
    let desc_start = embassy_time::Instant::now();
    let mut landed = false;
    loop {
        let elapsed = desc_start.elapsed().as_millis() as u64;
        if elapsed >= DESCENT_TIMEOUT_MS { break; }

        let sp = if elapsed < P3A_RAMP_MS {
            let f = elapsed as f32 / P3A_RAMP_MS as f32;
            TARGET_ALT * (1.0 - f) + P3A_FINAL * f
        } else if elapsed < P3A_RAMP_MS + P3B_RAMP_MS {
            let f = (elapsed - P3A_RAMP_MS) as f32 / P3B_RAMP_MS as f32;
            P3A_FINAL * (1.0 - f) + P3B_FINAL * f
        } else {
            P3B_FINAL
        };
        ALTITUDE_SETPOINT.signal(sp);

        if let Some(h) = lidar_rcv.try_get() {
            if h < GROUND_DETECT_M {
                let below = ground_since
                    .map(|t| t.elapsed().as_millis() as u64 >= GROUND_HOLD_MS)
                    .unwrap_or(false);
                if below {
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "[mission] P3: ground detected at h={:.2}m, disarming", h);
                    ulog::log(s.as_str());
                    landed = true;
                    break;
                }
                if ground_since.is_none() {
                    ground_since = Some(embassy_time::Instant::now());
                }
            } else {
                ground_since = None;
            }
        }

        Timer::after_millis(50).await;
    }
    if !landed {
        ulog::log("[mission] P3: descent timeout, disarming anyway");
    }
    ALTITUDE_SETPOINT.signal(0.0);

    // Log the flow-integrated body-frame displacement before disarming so
    // each flight report includes a dead-reckoned "how far did I land from
    // where I took off" number. Compare against the operator's tape-measure
    // reading to ground-truth the flow scale factor over time.
    let dx = FLOW_EST_X_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dy = FLOW_EST_Y_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dist = libm::sqrtf(dx * dx + dy * dy);
    let mut s: heapless::String<96> = heapless::String::new();
    let _ = write!(
        s,
        "[mission] flow_disp dx={:.2}m dy={:.2}m |d|={:.2}m (body frame, fwd/right)",
        dx, dy, dist
    );
    ulog::log(s.as_str());

    ulog::log("[mission] disarming");
    COMMAD_ARM_VEHICLE.send(false);

    ulog::log("[mission] test complete");
    loop {
        Timer::after_secs(60).await;
    }
}

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

#[embassy_executor::task]
async fn angle_to_rate_bridge() -> ! {
    let mut rcv = signals::ANGLE_TO_RATE_SP.receiver();
    let mut snd = signals::TRUE_RATE_SP.sender();
    loop {
        let [roll, pitch, _yaw] = rcv.changed().await;
        snd.send([roll, pitch, 0.0]);
    }
}

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
        Timer::after_millis(10).await;

        let t = embassy_time::Instant::now().as_millis();

        if cycle == 0 {
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
            let pid = pid_rcv.try_get();
            let rsp = ref_rcv.try_get();
            let thr = thr_rcv.try_get().unwrap_or(0.0);
            let att = att_rcv.try_get();

            if let Some(pid) = pid {
                let r = rsp.unwrap_or([0.0; 3]);
                let (rd, pd, yd) = match att {
                    Some(q) => {
                        let (r, p, y) = q.euler_angles();
                        (r.to_degrees(), p.to_degrees(), y.to_degrees())
                    }
                    None => (0.0, 0.0, 0.0),
                };
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

    const FLIP_COUNT_THRESHOLD: u32 = 10;
    // NED body frame (post override_imu_rot): level stationary drone reads
    // az = -g, fully inverted drone reads az = +g. Detect inverted by az
    // exceeding a positive threshold. The old `< -3.0` check was correct
    // for the previous chip-Z-up convention but fires immediately on boot
    // under NED (level az = -9.8 which passes the old threshold).
    const AZ_INVERTED_THRESHOLD: f32 = 3.0;

    let mut inverted_count: u32 = 0;

    loop {
        let d = rcv.changed().await;

        if d.acc[2] > AZ_INVERTED_THRESHOLD {
            inverted_count += 1;
            if inverted_count >= FLIP_COUNT_THRESHOLD {
                COMMAD_ARM_VEHICLE.send(false);
                ulog::log("[kill] FLIP DETECTED -- motors disarmed");

                for _ in 0..5 {
                    Timer::after_millis(200).await;
                    ulog::log("[kill] motors off (flip-kill)");
                }

                loop {
                    Timer::after_secs(60).await;
                }
            }
        } else {
            inverted_count = 0;
        }
    }
}

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

#[embassy_executor::task]
async fn flow_hold() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();

    let mut snd_att = signals::TRUE_ATTITUDE_Q_SP.sender();

    // KP_POS reverted 0.05 -> 0.0 after D000145. Confirmed in D000147 that
    // position hold was not the driver: with KP_POS=0 the drone still
    // oscillated at altitude and tripped the 5 rad/s gyro-runaway kill.
    // KD_VEL now lowered 0.25 -> 0.10 to reduce the flow-velocity -> tilt
    // gain. At D000147's ~0.5 m peak, flow reported vx = +/-0.8 m/s which
    // with KD_VEL=0.25 produced pitch commands at the +/-0.17 rad clamp.
    // With 0.10 the same input yields 0.08 rad (~4.6 deg) -- well below the
    // clamp and small enough that the body-rotation-driven feedback loop
    // cannot close. Proper structural fix is gyro-comp on the flow reading
    // (subtract omega*h), but this buys stability without changing shape.
    const KP_POS: f32 = 0.0;
    const KD_VEL: f32 = 0.10;
    const MAX_TILT_RAD: f32 = 0.17;

    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;

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

        const MAX_FLOW_VEL: f32 = 1.0;
        if vx.abs() > MAX_FLOW_VEL || vy.abs() > MAX_FLOW_VEL {
            continue;
        }

        let fq = FLOW_QUALITY.load(Ordering::Relaxed);
        if fq < 50 {
            continue;
        }

        const DT: f32 = 0.02;
        est_x += vx * DT;
        est_y += vy * DT;

        const POS_CLAMP: f32 = 0.5;
        let pos_ok = est_x.abs() < POS_CLAMP && est_y.abs() < POS_CLAMP;
        est_x = est_x.clamp(-POS_CLAMP, POS_CLAMP);
        est_y = est_y.clamp(-POS_CLAMP, POS_CLAMP);

        let kp = if pos_ok { KP_POS } else { 0.0 };

        let pitch_cmd = ( (kp * est_x + KD_VEL * vx)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll_cmd  = (-(kp * est_y + KD_VEL * vy)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        let q = UnitQuaternion::from_euler_angles(roll_cmd, pitch_cmd, 0.0);
        snd_att.send(q);

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

/// Integrate flow-reported body-frame velocity into a displacement estimate,
/// independent of `flow_hold` so we don't clamp at +/-0.5 m. Pure telemetry.
/// Uses actual elapsed time between samples rather than a nominal DT, so
/// dropped flow frames don't bias the integration.
#[embassy_executor::task]
async fn flow_position_logger() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();
    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;
    let mut last_t: Option<embassy_time::Instant> = None;
    loop {
        let [vx, vy] = rcv.changed().await;
        let now = embassy_time::Instant::now();
        if let Some(prev) = last_t {
            let dt = (now - prev).as_micros() as f32 / 1_000_000.0;
            // Sanity: reject absurd gaps (task paused, first sample after a
            // long idle, etc.) to keep the integration honest.
            if (0.001..0.5).contains(&dt) {
                est_x += vx * dt;
                est_y += vy * dt;
                FLOW_EST_X_MM.store((est_x * 1000.0) as i32, Ordering::Relaxed);
                FLOW_EST_Y_MM.store((est_y * 1000.0) as i32, Ordering::Relaxed);
            }
        }
        last_t = Some(now);
    }
}

/// Compute a tilt-compensated magnetic heading and log it alongside the
/// Madgwick-derived gyro yaw. Pure read-only telemetry tap: does not feed
/// into any controller. The mag is already calibrated (bias + soft-iron
/// matrix applied in `resources::compass_reader`). The `diff` column is
/// the quantity to watch: if it stays roughly constant over a flight, the
/// gyro-based yaw is not drifting. If it walks, we have empirical
/// quantification of gyro-yaw drift rate.
///
/// Field magnitude |B| is logged as a sanity check. Earth's field indoors
/// typically reads 25-60 uT depending on latitude and shielding; if |B|
/// jumps with throttle that's motor-current interference on the mag.
#[embassy_executor::task]
async fn mag_yaw_logger() -> ! {
    // Bug fix 2026-04-23: this task was originally reading CAL_MAG_DATA,
    // which is declared in common::signals but nothing writes to it on this
    // board. The device-side compass_reader publishes the calibrated mag
    // vector to CAL_MULTI_MAG_DATA[0] (a multi_watch of [f32;3]). That's
    // what we actually need to subscribe to. D000079-D000082 produced zero
    // [mag] log lines because of this mismatch.
    let mut mag_rcv = common::signals::CAL_MULTI_MAG_DATA[0].receiver();
    let mut att_rcv = signals::AHRS_ATTITUDE_Q.receiver();
    loop {
        Timer::after_millis(500).await;

        let Some(mag) = mag_rcv.try_get() else { continue };
        let Some(q) = att_rcv.try_get() else { continue };

        let (roll, pitch, yaw_gyro_rad) = q.euler_angles();
        let yaw_gyro = yaw_gyro_rad.to_degrees();

        let mx = mag[0];
        let my = mag[1];
        let mz = mag[2];
        let b_mag = libm::sqrtf(mx * mx + my * my + mz * mz);

        // Tilt-compensated heading for body frame (x-forward, y-right,
        // z-down). Standard formula (see Honeywell AN-203):
        //   Xh = mx*cos(p) + my*sin(r)*sin(p) + mz*cos(r)*sin(p)
        //   Yh = my*cos(r) - mz*sin(r)
        //   yaw = atan2(-Yh, Xh)
        // atan2(-Yh, Xh) returns yaw positive CW viewed from above, matching
        // NED body-frame yaw convention. If the drone is actually aligned
        // with magnetic north at arm time, yaw_mag == 0.
        let cr = libm::cosf(roll);
        let sr = libm::sinf(roll);
        let cp = libm::cosf(pitch);
        let sp = libm::sinf(pitch);
        let xh = mx * cp + my * sr * sp + mz * cr * sp;
        let yh = my * cr - mz * sr;
        let yaw_mag = libm::atan2f(-yh, xh).to_degrees();

        // Wrap diff to (-180, 180] so growing drift is readable as a
        // continuously-changing value rather than a ±360 jump.
        let mut diff = yaw_mag - yaw_gyro;
        while diff > 180.0 { diff -= 360.0; }
        while diff < -180.0 { diff += 360.0; }

        let mut s: heapless::String<96> = heapless::String::new();
        let _ = write!(
            s,
            "[mag] yaw_mag={:.1} yaw_gyro={:.1} diff={:.1} |B|={:.1}uT",
            yaw_mag, yaw_gyro, diff, b_mag
        );
        ulog::log(s.as_str());
    }
}

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

    let mut b = [0u8; 1];
    let mut hdr = [0u8; 6];
    let mut pbuf = [0u8; mtf01::MAX_PAYLOAD + 1];

    loop {
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

        uart.read(&mut hdr).await.ok();
        let size = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if size > mtf01::MAX_PAYLOAD {
            continue;
        }

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
                // Gate on 25 cm AND quality >= 60.
                // Raised 10 cm -> 25 cm after D000026 flipped the drone at
                // ~20 cm during descent: flow_hold's tilt commands (+/-4-6
                // deg from velocity-induced-by-body-rotation noise) destabi-
                // lise the drone in the final landing approach, where the
                // drone cannot tolerate any lateral excursion without tip-
                // ping into a prop-down crash. Silencing flow below 25 cm
                // means the last metre of descent is attitude-only control,
                // which is stable enough to land without wobbling. The 25 cm
                // floor is also above the typical lidar minimum-range stuck
                // regime (D000009), so flow genuinely stops when descent
                // begins rather than receiving noisy data near ground.
                // Quality >= 60 threshold unchanged from D000136 fix.
                if f.quality > 60 && last_height_m > 0.25 {
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
