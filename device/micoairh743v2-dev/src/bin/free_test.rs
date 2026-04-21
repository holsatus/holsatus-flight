//! MicoAir H743 -- thrust staircase test binary.
//!
//! Built on flight.rs task structure verbatim (known working reference).
//! The only difference from flight.rs is the mission body: instead of the
//! altitude-hold climb/hover/descend sequence, this binary executes a
//! 6-step thrust staircase for motor and controller characterisation.
//!
//! Mission:
//!   3 thrust steps: 90, 105, 120 percent of HOVER_THRUST
//!   Each step: 3 s ramp up, 10 s hold, 2 s ramp down, 3 s gap
//!   Total arm-to-disarm: ~54 s
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
    // flow_hold DISABLED for staircase tests. In D000049 the drone held
    // attitude within ~1.5 deg for 1.5 s of step 2 hold, then lifted just
    // above the 5 cm lidar threshold, flow_hold activated, and its position-
    // error-to-tilt mapping formed a positive feedback loop at the borderline
    // hover regime: small motion -> large tilt command -> larger motion ->
    // larger tilt. flow_hold is intended for sustained altitude hold where
    // the drone is truly airborne, not brief staircase liftoff transients.
    // thread_spawner.spawn(flow_hold().unwrap());
    thread_spawner.spawn(flip_kill().unwrap());
    thread_spawner.spawn(gyro_runaway_kill().unwrap());
    thread_spawner.spawn(micoairh743v2::rc_kill::rc_kill_task(r.rc).unwrap());
    thread_spawner.spawn(staircase_mission().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());

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

    const HOVER_THRUST: f32 = 4.50;
    // 4-step staircase. D000064-D000093 established the 0.90-1.20 range;
    // D000105 showed all three stairs completed without liftoff on a pack
    // that was already partially discharged, so an extra 1.35 step was
    // added on top to guarantee a liftoff observation when the pack is
    // fresh. D000063 (old heavier Kapla gear) showed yaw-spin at 1.20,
    // so 1.35 is the upper bound that still leaves dial-down room if the
    // drone lifts cleanly on step 2 or 3.
    const STEPS: [f32; 4] = [0.90, 1.05, 1.20, 1.35];
    // Fast ramp-up: traverse the stick-slip ground-contact zone (~85-105% of
    // actual hover thrust) before friction builds a motor-differential load
    // large enough to snap into yaw runaway when the leg releases. D000108/
    // D000110 both failed at step 1 (104%) with RAMP_UP_S=3, which spent
    // ~0.57 s in that band; 1 s shrinks it to ~0.19 s and is robust to
    // pack-voltage drift that would otherwise shift the bad zone onto
    // whichever step sits at the instantaneous hover throttle.
    const RAMP_UP_S: u64 = 1;
    // HOLD_S extended from 5 s to 10 s in D000048+. D000047 tipped during
    // the liftoff-regime step (1.50) roughly 1-2 s into the hold. Giving the
    // angle controller twice as long to settle at each thrust level provides
    // more headroom to observe whether a near-balanced drone stabilises in
    // free flight or diverges at a predictable rate.
    const HOLD_S: u64 = 10;
    const RAMP_DN_S: u64 = 2;
    const GAP_S: u64 = 3;

    for (idx, &frac) in STEPS.iter().enumerate() {
        let target = HOVER_THRUST * frac;
        let pct = (frac * 100.0) as u32;

        let mut s: heapless::String<48> = heapless::String::new();
        let _ = write!(s, "[mission] step={} frac={}% target={:.2}", idx, pct, target);
        ulog::log(s.as_str());

        let ramp_start = embassy_time::Instant::now();
        loop {
            let elapsed = ramp_start.elapsed().as_millis();
            if elapsed >= RAMP_UP_S * 1000 { break; }
            let f = elapsed as f32 / (RAMP_UP_S * 1000) as f32;
            signals::TRUE_Z_THRUST_SP.send(target * f);
            Timer::after_millis(20).await;
        }

        let hold_start = embassy_time::Instant::now();
        while hold_start.elapsed().as_millis() < HOLD_S * 1000 {
            signals::TRUE_Z_THRUST_SP.send(target);
            Timer::after_millis(20).await;
        }

        let dn_start = embassy_time::Instant::now();
        loop {
            let elapsed = dn_start.elapsed().as_millis();
            if elapsed >= RAMP_DN_S * 1000 { break; }
            let f = 1.0 - elapsed as f32 / (RAMP_DN_S * 1000) as f32;
            signals::TRUE_Z_THRUST_SP.send(target * f);
            Timer::after_millis(20).await;
        }
        signals::TRUE_Z_THRUST_SP.send(0.0);

        Timer::after_secs(GAP_S).await;
    }

    ulog::log("[mission] staircase complete, disarming");
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

    const KP_POS: f32 = 0.30;
    const KD_VEL: f32 = 0.25;
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
                if f.quality > 30 && last_height_m > 0.05 {
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
