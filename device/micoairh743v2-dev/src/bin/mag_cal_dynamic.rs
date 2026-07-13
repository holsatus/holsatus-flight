//! MicoAir H743v2 -- QMC5883L magnetometer calibration WITH motors spinning.
//!
//! Purpose:
//!   Collect mag-cal samples under the same EMI/current conditions the
//!   Madgwick filter actually sees in flight. The static mag_cal.rs binary
//!   with silent ESCs yields a cal that matches a bench environment,
//!   whereas the compass in-flight sees hard-iron offsets and time-varying
//!   fields from the 4 motor phases, the ESC DC bus, and current return
//!   paths through the flight controller ground plane. A cal fit from
//!   dynamic data reproduces those conditions and should give a heading
//!   estimate that actually tracks during powered flight.
//!
//! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! !!                              SAFETY                                  !!
//! !!                                                                      !!
//! !!  PROPELLERS MUST BE OFF BEFORE POWERING THE DRONE WITH THIS BINARY.  !!
//! !!                                                                      !!
//! !!  All four motors WILL spin up under DShot throttle. The binary also  !!
//! !!  ramps rather than steps the throttle so that if a prop was left on  !!
//! !!  by accident the operator has a few seconds to cut power.            !!
//! !!                                                                      !!
//! !!  Secure the drone in a vise / clamp / heavy padded box -- motor      !!
//! !!  torque without props will still spin the frame, and a loose drone   !!
//! !!  will wander.                                                        !!
//! !!                                                                      !!
//! !!  Keep your hands OFF the motor shafts once armed. Bare hex shafts    !!
//! !!  can draw in fingers/hair.                                           !!
//! !!                                                                      !!
//! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!
//! Procedure (after confirming props off + drone secured):
//!   1. Flash this binary.
//!   2. Power the drone. Watch the LEDs:
//!        * 3 quick green flashes = boot ok
//!        * Red+green alternating at 1 Hz (3 s) = ABORT WINDOW. If props
//!          are on or the drone is loose, cut power NOW. Motors are still
//!          silent (DShot-0) during this window.
//!        * Solid green = MOTORS ARMING (5 s DShot-0 stream for ESC handshake).
//!        * Solid blue = MOTORS RAMPING (3 s: 0 -> RAMP_DSHOT). Throttle
//!          grows slowly so you can still cut power if something looks off.
//!        * Blue pulsing fast (2 Hz) = COLLECTING (COLLECT_S s). Motors
//!          spinning at RAMP_DSHOT. Slowly rotate the drone-in-rig
//!          (<= 90 deg/s) through orientations. Budget as in mag_cal.rs:
//!             - First 60 s: hold each of the 6 faces up for 10 s,
//!               rotating slowly around the vertical body axis per hold.
//!             - Next 60 s: figure-8s, rolls, pitches in-hand.
//!             - Last 60 s: repeat whatever coverage looked thin.
//!          UART prints a 10 s countdown so you can pace yourself.
//!          Stay in one spot; do not walk past laptops/fridges/steel.
//!        * Solid green = MOTORS SPINNING DOWN (2 s: RAMP_DSHOT -> 0).
//!        * Green+blue alternating at 5 Hz (rapid flicker) = DONE. Motors
//!          stopped, file flushed, safe to power off.
//!        * Solid red / red blinking = error (see UART for reason).
//!   3. Pull the SD card; the log is in D<NNNNNN>/000001.CSV.
//!   4. Run tools/mag_cal_fit.py on the CSV to compute hard/soft-iron cal.
//!   5. Paste the offsets + scale matrix into resources.rs (MAG_BIAS_UT +
//!      MAG_CAL_MAT) for the flight binary to pick up on next flash.
//!
//! Hardware:
//!   QMC5883L  I2C2  SCL=PB10 SDA=PB11  addr=0x0D
//!   Motors    TIM1  M1=PE9 M2=PE11 M3=PE13 M4=PE14 (DShot300, DMA1_CH1)
//!   SDMMC1          CLK=PC12 CMD=PD2  D0-D3=PC8-PC11
//!   UART1           TX=PA9  115 200 baud
//!   LEDs            red=PE3 blue=PE4 green=PE2
//!
//! CSV format (identical to mag_cal.rs so the existing fit script works):
//!   t_us,mx_raw,my_raw,mz_raw,mx_uT,my_uT,mz_uT
//!     t_us: monotonic microseconds from boot
//!     mx/my/mz_raw: signed 16-bit LSB values from QMC5883L
//!     mx/my/mz_uT: raw * 0.244 uT/LSB (2 Gauss range)

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{DMA1_CH1, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4, Channel};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::Peri;
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::qmc5883l::{Qmc5883l, ADDR as QMC_ADDR};
use micoairh743v2::resources::{I2c2Irqs, MotorIrqs};
use micoairh743v2::sdlog::SdmmcResources;
use micoairh743v2::resources::UartLogIrqs;

/// QMC5883L sensitivity at 2 Gauss range.
const SENSITIVITY_UT_PER_LSB: f32 = 0.244;

/// DShot throttle during the capture phase. Hover on this frame is
/// ~750 DShot; 400 is safely below hover but still drives real motor
/// current / EMI so the cal captures the in-flight magnetic environment.
/// Do NOT raise this without revisiting prop-off safety procedure.
const RAMP_DSHOT: u16 = 400;

/// Operator abort window (props visibly off? drone secured?) before
/// motors start arming. Motors are silent during this window.
const ABORT_WINDOW_S: u64 = 3;

/// DShot-0 stream duration for ESC arming handshake.
const ARM_S: u64 = 5;

/// Throttle ramp-up duration (0 -> RAMP_DSHOT). A slow ramp gives the
/// operator one last window to cut power if a prop was left on.
const RAMP_UP_S: u64 = 3;

/// Throttle ramp-down duration (RAMP_DSHOT -> 0). Short is fine; motors
/// decelerate faster than the ramp on their own under no load.
const RAMP_DOWN_S: u64 = 2;

/// Mag capture duration at RAMP_DSHOT. 180 s matches mag_cal.rs so the
/// post-processing script sees the same sample count per run.
const COLLECT_S: u64 = 180;

/// Mag sample period (50 Hz).
const SAMPLE_PERIOD_MS: u64 = 20;

/// DShot frame slots (16 data bits + tail padding).
const SLOTS: usize = 24;


fn encode(throttle: u16, telemetry: bool) -> u16 {
    let value = (throttle << 1) | (telemetry as u16);
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    (value << 4) | crc
}

fn build_frame(packet: u16, b0: u16, b1: u16) -> [u16; SLOTS] {
    let mut slots = [0u16; SLOTS];
    for i in 0..16 {
        let bit = (packet >> (15 - i)) & 1;
        slots[i] = if bit == 1 { b1 } else { b0 };
    }
    slots
}

async fn send_all(
    pwm: &mut SimplePwm<'_, TIM1>,
    dma: &mut Peri<'_, DMA1_CH1>,
    frame: &[u16; SLOTS],
) {
    let channels = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
    for ch in channels {
        pwm.waveform_up_multi_channel(dma.reborrow(), MotorIrqs, ch, ch, frame)
            .await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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
    uart.write(b"mag_cal_dynamic: UART ok\r\n").await.ok();
    uart.write(b"mag_cal_dynamic: PROPS OFF + DRONE SECURED?\r\n").await.ok();

    // ── I2C2 + QMC5883L init ─────────────────────────────────────────
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;
    let mut i2c = I2c::new(
        p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, I2c2Irqs, i2c_cfg,
    );

    if i2c.write(QMC_ADDR, &[]).await.is_ok() {
        uart.write(b"mag_cal_dynamic: QMC5883L found at 0x0D\r\n").await.ok();
    } else {
        uart.write(b"mag_cal_dynamic: NOTHING at 0x0D -- check wiring\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }

    let mut mag = Qmc5883l::new(i2c);
    if mag.init().await.is_err() {
        uart.write(b"mag_cal_dynamic: QMC5883L init FAILED\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }
    uart.write(b"mag_cal_dynamic: QMC5883L init ok\r\n").await.ok();

    // ── SDMMC1 init + FAT mount + session dir creation ───────────────
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
                let mut s: String<64> = String::new();
                write!(s, "mag_cal_dynamic: SD attempt {} FAIL\r\n", attempt).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"mag_cal_dynamic: SD FAIL (giving up)\r\n").await.ok();
        fatal_blink(&mut led_red).await;
    }

    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"mag_cal_dynamic: FAT mount FAIL\r\n").await.ok();
            fatal_blink(&mut led_red).await;
        }
    };

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

    file.write_all(b"t_us,mx_raw,my_raw,mz_raw,mx_uT,my_uT,mz_uT\r\n").await.ok();
    file.flush().await.ok();

    let mut m: String<64> = String::new();
    write!(m, "mag_cal_dynamic: logging to {}/000001.CSV\r\n", dir_name.as_str()).ok();
    uart.write(m.as_bytes()).await.ok();

    // ── Motor PWM setup ──────────────────────────────────────────────
    let ch1 = PwmPin::<_, Ch1>::new(p.PE9, OutputType::PushPull);
    let ch2 = PwmPin::<_, Ch2>::new(p.PE11, OutputType::PushPull);
    let ch3 = PwmPin::<_, Ch3>::new(p.PE13, OutputType::PushPull);
    let ch4 = PwmPin::<_, Ch4>::new(p.PE14, OutputType::PushPull);

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1),
        Some(ch2),
        Some(ch3),
        Some(ch4),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.ch1().max_duty_cycle() as u32;
    let b0 = ((max_duty * 384) >> 10) as u16;
    let b1 = ((max_duty * 768) >> 10) as u16;
    let disarm_frame = build_frame(encode(0, false), b0, b1);
    let mut dma = p.DMA1_CH1;

    // ── Phase A: ABORT WINDOW (motors silent) ────────────────────────
    uart.write(b"mag_cal_dynamic: ABORT WINDOW -- props off? drone secured?\r\n").await.ok();
    for _ in 0..ABORT_WINDOW_S {
        led_red.set_high();
        led_green.set_low();
        Timer::after_millis(500).await;
        led_red.set_low();
        led_green.set_high();
        Timer::after_millis(500).await;
    }
    led_red.set_low();
    led_green.set_low();

    // ── Phase B: ARM ESCs (DShot-0 stream, solid green) ──────────────
    uart.write(b"mag_cal_dynamic: arming ESCs\r\n").await.ok();
    led_green.set_high();
    let arm_start = Instant::now();
    while arm_start.elapsed() < Duration::from_secs(ARM_S) {
        send_all(&mut pwm, &mut dma, &disarm_frame).await;
        Timer::after_micros(500).await;
    }
    led_green.set_low();

    // ── Phase C: RAMP UP (solid blue) ────────────────────────────────
    let mut s: String<64> = String::new();
    write!(s, "mag_cal_dynamic: ramping 0 -> {}\r\n", RAMP_DSHOT).ok();
    uart.write(s.as_bytes()).await.ok();
    led_blue.set_high();
    let ramp_start = Instant::now();
    loop {
        let e_ms = ramp_start.elapsed().as_millis() as u32;
        let total_ms = (RAMP_UP_S as u32) * 1000;
        if e_ms >= total_ms {
            break;
        }
        // Linear ramp from 48 (min valid DShot throttle) to RAMP_DSHOT.
        let thr = 48 + ((RAMP_DSHOT - 48) as u32 * e_ms / total_ms) as u16;
        let frame = build_frame(encode(thr, false), b0, b1);
        send_all(&mut pwm, &mut dma, &frame).await;
        Timer::after_micros(500).await;
    }
    led_blue.set_low();

    // ── Phase D: COLLECT (motors at RAMP_DSHOT, blue 2 Hz blink) ─────
    let mut m: String<96> = String::new();
    write!(m, "mag_cal_dynamic: COLLECTING {}s at DShot={} -- rotate slowly\r\n",
           COLLECT_S, RAMP_DSHOT).ok();
    uart.write(m.as_bytes()).await.ok();

    let capture_frame = build_frame(encode(RAMP_DSHOT, false), b0, b1);

    let collect_start = Instant::now();
    let mut last_blink = Instant::now();
    let mut last_sample = Instant::now();
    let mut blink_on = false;
    let mut sample_count: u32 = 0;
    let mut next_countdown_s: u64 = COLLECT_S;

    while collect_start.elapsed() < Duration::from_secs(COLLECT_S) {
        // Keep DShot stream alive every iteration (~500 us cadence).
        send_all(&mut pwm, &mut dma, &capture_frame).await;

        if last_blink.elapsed() > Duration::from_millis(250) {
            blink_on = !blink_on;
            if blink_on {
                led_blue.set_high();
            } else {
                led_blue.set_low();
            }
            last_blink = Instant::now();
        }

        let remaining_s = COLLECT_S.saturating_sub(collect_start.elapsed().as_secs());
        if remaining_s < next_countdown_s && remaining_s.is_multiple_of(10) {
            let mut c: String<48> = String::new();
            write!(c, "mag_cal_dynamic: {}s remaining\r\n", remaining_s).ok();
            uart.write(c.as_bytes()).await.ok();
            next_countdown_s = remaining_s;
        }

        if last_sample.elapsed() >= Duration::from_millis(SAMPLE_PERIOD_MS) {
            last_sample = Instant::now();
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

                    if sample_count.is_multiple_of(10) {
                        file.flush().await.ok();
                    }
                }
                Err(_) => {
                    uart.write(b"mag_cal_dynamic: mag read err\r\n").await.ok();
                }
            }
        }

        Timer::after_micros(500).await;
    }

    // ── Phase E: RAMP DOWN (solid green) ─────────────────────────────
    uart.write(b"mag_cal_dynamic: ramping down\r\n").await.ok();
    led_blue.set_low();
    led_green.set_high();
    let ramp_down_start = Instant::now();
    loop {
        let e_ms = ramp_down_start.elapsed().as_millis() as u32;
        let total_ms = (RAMP_DOWN_S as u32) * 1000;
        if e_ms >= total_ms {
            break;
        }
        let thr = RAMP_DSHOT - ((RAMP_DSHOT - 48) as u32 * e_ms / total_ms) as u16;
        let frame = build_frame(encode(thr, false), b0, b1);
        send_all(&mut pwm, &mut dma, &frame).await;
        Timer::after_micros(500).await;
    }

    // ── Phase F: DISARM (DShot-0 stream) ─────────────────────────────
    for _ in 0u32..2000 {
        send_all(&mut pwm, &mut dma, &disarm_frame).await;
        Timer::after_micros(500).await;
    }
    led_green.set_low();

    // ── Phase G: DONE ────────────────────────────────────────────────
    file.flush().await.ok();
    drop(file);

    let mut m: String<80> = String::new();
    write!(m, "mag_cal_dynamic: DONE -- {} samples written, safe to power off\r\n",
           sample_count).ok();
    uart.write(m.as_bytes()).await.ok();

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
