//! MicoAir H743v2 -- QMC5883L magnetometer calibration logger.
//!
//! Logs raw QMC5883L readings at 50 Hz to a CSV file on the SD card for
//! offline hard-iron / soft-iron calibration fitting. No motors, no
//! controllers: the ESC silence task sends continuous DShot-0 frames to
//! keep the motor ESCs quiet regardless of throttle.
//!
//! Procedure (props removed for extra safety):
//!   1. Flash this binary.
//!   2. Power the drone. Watch the LEDs:
//!        * 3 quick green flashes = boot ok
//!        * Green pulsing slowly (1 Hz) = GET READY (5 s window). Hold
//!          the drone in your hand in a fixed starting pose.
//!        * Blue pulsing fast (2 Hz) = COLLECTING (180 s). Slowly rotate
//!          (<= 90 deg/s) through orientations. Budget the 180 s as:
//!             - First 90 s: hold each of the 6 faces up for 15 s,
//!               rotating the drone slowly around the vertical body
//!               axis during each hold (one full revolution per hold).
//!               Do top and BELLY-UP first so cardinal coverage is
//!               guaranteed even if you run long.
//!             - Next 75 s: horizontal figure-8s, full body rolls,
//!               full body pitches. These fill off-cardinal regions.
//!             - Last 15 s: whatever gaps you noticed in the mirror.
//!          UART prints a 10 s countdown so you can pace yourself.
//!          Stay in one spot; do not walk past laptops/fridges/steel.
//!        * Green+blue alternating at 5 Hz (rapid flicker) = DONE. Safe
//!          to power off (pull LiPo). Data is only guaranteed on the SD
//!          card after this signal -- the directory entry is committed
//!          when the file closes here. This pattern is visually distinct
//!          from the solid-blue-ish blink during collection so there is
//!          no ambiguity about whether sampling is still running.
//!        * Solid red / red blinking = error (see UART for reason).
//!   3. Pull the SD card; the log is in D<NNNNNN>/000001.CSV.
//!   4. Run the post-processing script to compute hard-iron + soft-iron
//!      calibration (see tools/mag_cal_fit.py -- provided alongside this
//!      binary).
//!   5. Paste the resulting offsets + scale factors into the compass
//!      reader so subsequent flights get absolute yaw reference.
//!
//! Hardware:
//!   QMC5883L  I2C2  SCL=PB10 SDA=PB11  addr=0x0D
//!   SDMMC1          CLK=PC12 CMD=PD2  D0-D3=PC8-PC11
//!   UART1           TX=PA9  115 200 baud
//!   LEDs            red=PE3 blue=PE4 green=PE2
//!
//! CSV format:
//!   t_us,mx_raw,my_raw,mz_raw,mx_uT,my_uT,mz_uT
//!     t_us: monotonic microseconds from boot
//!     mx/my/mz_raw: signed 16-bit LSB values from QMC5883L
//!     mx/my/mz_uT: raw * 0.244 uT/LSB (2 Gauss range)

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::qmc5883l::{Qmc5883l, ADDR as QMC_ADDR};
use micoairh743v2::resources::I2c2Irqs;
use micoairh743v2::sdlog::SdmmcResources;
use micoairh743v2::resources::UartLogIrqs;

/// QMC5883L sensitivity at 2 Gauss range.
const SENSITIVITY_UT_PER_LSB: f32 = 0.244;

/// Time to give the user to get the drone into its starting pose.
const GET_READY_S: u64 = 5;

/// Total collection duration. 180 s at 50 Hz = ~9000 samples. Budget:
/// 90 s for six face-holds (with slow in-plane yaw during each hold),
/// 75 s for figure-8s, rolls, pitches to fill off-cardinal coverage,
/// 15 s spare to fill whatever gaps remained.
const COLLECT_S: u64 = 180;

/// Sample period in milliseconds.
const SAMPLE_PERIOD_MS: u64 = 20;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_red = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    // Boot flash: 3 quick greens.
    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    let mut uart = UartTx::new(
        p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default(),
    )
    .unwrap();
    uart.write(b"mag_cal: UART ok\r\n").await.ok();

    // Keep motors quiet regardless of props-on/off state.
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());
    uart.write(b"mag_cal: ESC silence task started\r\n").await.ok();

    // I2C2 + QMC5883L init.
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;
    let mut i2c = I2c::new(
        p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, I2c2Irqs, i2c_cfg,
    );

    // Prime the DMA path with a probe write before the driver reads.
    if i2c.write(QMC_ADDR, &[]).await.is_ok() {
        uart.write(b"mag_cal: QMC5883L found at 0x0D\r\n").await.ok();
    } else {
        uart.write(b"mag_cal: NOTHING at 0x0D -- check wiring\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }

    let mut mag = Qmc5883l::new(i2c);
    if mag.init().await.is_err() {
        uart.write(b"mag_cal: QMC5883L init FAILED\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }
    uart.write(b"mag_cal: QMC5883L init ok\r\n").await.ok();

    // SDMMC1 init + FAT mount + session dir creation.
    let mut device = SdmmcResources {
        periph: p.SDMMC1,
        clk: p.PC12,
        cmd: p.PD2,
        d0: p.PC8,
        d1: p.PC9,
        d2: p.PC10,
        d3: p.PC11,
    }
    .setup();

    let mut sd_ok = false;
    for attempt in 1u8..=5 {
        match device.try_reset().await {
            Ok(()) => {
                sd_ok = true;
                break;
            }
            Err(_) => {
                let mut s: String<48> = String::new();
                write!(s, "mag_cal: SD attempt {} FAIL\r\n", attempt).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"mag_cal: SD FAIL (giving up)\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }

    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"mag_cal: FAT mount FAIL\r\n").await.ok();
            fatal_blink(&mut led_red).await;
        }
    };

    // Pick next unused session dir so we do not overwrite existing logs.
    let mut session_idx: u32 = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        if entry.is_dir() {
            if let Some(idx) = parse_session_dir_idx(entry.short_file_name_as_bytes()) {
                session_idx = session_idx.max(idx);
            }
        }
    }
    session_idx += 1;

    let mut dir_name: String<8> = String::new();
    write!(dir_name, "D{:06}", session_idx).ok();
    let session_dir = fs
        .root_dir()
        .create_dir(dir_name.as_str())
        .await
        .unwrap_or_else(|_| loop {});

    let mut file = session_dir
        .create_file("000001.CSV")
        .await
        .unwrap_or_else(|_| loop {});

    // Write header row.
    file.write_all(b"t_us,mx_raw,my_raw,mz_raw,mx_uT,my_uT,mz_uT\r\n").await.ok();
    file.flush().await.ok();

    let mut m: String<48> = String::new();
    write!(m, "mag_cal: logging to {}/000001.CSV\r\n", dir_name.as_str()).ok();
    uart.write(m.as_bytes()).await.ok();

    // ── Phase 1: GET READY (slow green blink for GET_READY_S) ────────
    uart.write(b"mag_cal: GET READY -- hold drone in starting pose\r\n").await.ok();
    for _ in 0..GET_READY_S {
        led_green.set_high();
        Timer::after_millis(500).await;
        led_green.set_low();
        Timer::after_millis(500).await;
    }

    // ── Phase 2: COLLECT (fast blue blink for COLLECT_S) ─────────────
    let mut m: String<80> = String::new();
    write!(m, "mag_cal: COLLECTING {}s -- rotate slowly through all orientations\r\n", COLLECT_S).ok();
    uart.write(m.as_bytes()).await.ok();

    let start = Instant::now();
    let mut last_blink = Instant::now();
    let mut blink_on = false;
    let mut sample_count: u32 = 0;
    let mut next_countdown_s: u64 = COLLECT_S;

    while start.elapsed() < Duration::from_secs(COLLECT_S) {
        // 2 Hz blue blink during collection.
        if last_blink.elapsed() > Duration::from_millis(250) {
            blink_on = !blink_on;
            if blink_on {
                led_blue.set_high();
            } else {
                led_blue.set_low();
            }
            last_blink = Instant::now();
        }

        // Countdown on UART every 10 s so operator can pace rotation.
        let remaining_s = COLLECT_S.saturating_sub(start.elapsed().as_secs());
        if remaining_s < next_countdown_s && remaining_s.is_multiple_of(10) {
            let mut c: String<48> = String::new();
            write!(c, "mag_cal: {}s remaining\r\n", remaining_s).ok();
            uart.write(c.as_bytes()).await.ok();
            next_countdown_s = remaining_s;
        }

        match mag.read().await {
            Ok(d) => {
                let t_us = Instant::now().as_micros();
                let ux = d.x as f32 * SENSITIVITY_UT_PER_LSB;
                let uy = d.y as f32 * SENSITIVITY_UT_PER_LSB;
                let uz = d.z as f32 * SENSITIVITY_UT_PER_LSB;
                let mut row: String<128> = String::new();
                write!(
                    row,
                    "{},{},{},{},{:.2},{:.2},{:.2}\r\n",
                    t_us, d.x, d.y, d.z, ux, uy, uz
                )
                .ok();
                file.write_all(row.as_bytes()).await.ok();
                sample_count += 1;

                // Flush every ~200 ms so a power loss during COLLECT still
                // leaves a usable CSV with at most 10 samples of tail loss.
                if sample_count.is_multiple_of(10) {
                    file.flush().await.ok();
                }
            }
            Err(_) => {
                uart.write(b"mag_cal: read err\r\n").await.ok();
            }
        }

        Timer::after_millis(SAMPLE_PERIOD_MS).await;
    }

    // ── Phase 3: DONE ────────────────────────────────────────────────
    file.flush().await.ok();
    drop(file);
    led_blue.set_low();
    led_green.set_low();

    let mut m: String<64> = String::new();
    write!(m, "mag_cal: DONE -- {} samples written, safe to power off\r\n", sample_count).ok();
    uart.write(m.as_bytes()).await.ok();

    // Green+blue alternating at 5 Hz forever. Distinct from the lonely
    // blue blink during collection so the operator can tell at a glance
    // that sampling has stopped and the file is safely flushed.
    loop {
        led_green.set_high();
        led_blue.set_low();
        Timer::after_millis(100).await;
        led_green.set_low();
        led_blue.set_high();
        Timer::after_millis(100).await;
    }
}

async fn fatal_blink(led: &mut Output<'_>) -> ! {
    loop {
        led.set_high();
        Timer::after_millis(200).await;
        led.set_low();
        Timer::after_millis(200).await;
    }
}

/// Parse session index from a D%06u FAT 8.3 short name.
fn parse_session_dir_idx(name: &[u8]) -> Option<u32> {
    if name.len() != 7 {
        return None;
    }
    if name[0] != b'D' && name[0] != b'd' {
        return None;
    }
    let mut idx: u32 = 0;
    for &b in &name[1..] {
        if b < b'0' || b > b'9' {
            return None;
        }
        idx = idx * 10 + (b - b'0') as u32;
    }
    Some(idx)
}
