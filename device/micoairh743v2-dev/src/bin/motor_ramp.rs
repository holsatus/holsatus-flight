//! Motor ramp test with SD card logging.
//!
//! Ramps all four motors from idle to RAMP_MAX, holds, then ramps down.
//! Logs throttle + IMU data to SD card at 50 Hz for post-flight analysis.
//! No attitude controller -- just equal thrust on all motors.
//!
//! Sequence:
//!   0-5 s:   arm ESCs (DShot-0)
//!   5-20 s:  ramp from throttle 48 to RAMP_MAX
//!   20-25 s: hold at RAMP_MAX
//!   25-30 s: ramp down to 48
//!   30 s:    disarm
//!
//! Adjust RAMP_MAX below. Hover DShot for this frame is ~450.

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH1, TIM1};
use embassy_stm32::spi::{self, mode::Master as SpiMaster, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::Peri;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::String;
use static_cell::StaticCell;
use micoairh743v2::bmi088::Bmi088;
use micoairh743v2::resources::{MotorIrqs, Spi2Irqs};
use micoairh743v2::sdlog::SdmmcResources;
use micoairh743v2::resources::UartLogIrqs;
use {defmt_rtt as _, panic_probe as _};

#[path = "../config.rs"]
mod config;


/// Maximum throttle. Hover for this frame is ~450 DShot.
const RAMP_MAX: u16 = 150;

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
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let _led_red      = Output::new(p.PE3, Level::Low, Speed::Low);

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default()).unwrap();

    // ── BMI088 IMU setup (SPI2) ──────────────────────────────────────────────
    let cs_acc = Output::new(p.PD4, Level::High, Speed::High);
    let cs_gyr = Output::new(p.PD5, Level::High, Speed::High);
    let spi2_cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(8_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = embassy_stm32::gpio::Pull::Up;
        c
    };
    let spi2 = Spi::new(p.SPI2, p.PD3, p.PC3, p.PC2, p.DMA1_CH6, p.DMA1_CH7, Spi2Irqs, spi2_cfg);
    type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;
    static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi2_bus: &'static Spi2Bus = SPI2_BUS.init(Mutex::new(spi2));

    let accel_dev = SpiDeviceWithConfig::new(spi2_bus, cs_acc, spi2_cfg);
    let gyro_dev  = SpiDeviceWithConfig::new(spi2_bus, cs_gyr, spi2_cfg);
    let mut imu = Bmi088::new(accel_dev, gyro_dev);

    for attempt in 1u8..=5 {
        match imu.init().await {
            Ok(()) => {
                let _ = uart.write(b"motor_ramp: BMI088 ok\r\n").await;
                break;
            }
            Err(_) if attempt < 5 => Timer::after_millis(100).await,
            Err(_) => { let _ = uart.write(b"motor_ramp: BMI088 FAIL\r\n").await; }
        }
    }

    // ── SD card setup (best-effort) ──────────────────────────────────────────
    // SD setup -- keep all objects in main scope for lifetime reasons.
    let mut device = SdmmcResources {
        periph: p.SDMMC1, clk: p.PC12, cmd: p.PD2,
        d0: p.PC8, d1: p.PC9, d2: p.PC10, d3: p.PC11,
    }.setup();

    let _ = uart.write(b"motor_ramp: SD init...\r\n").await;
    let mut sd_ok = false;
    for _ in 0u8..3 {
        if device.try_reset().await.is_ok() { sd_ok = true; break; }
        Timer::after_millis(500).await;
    }

    let stream = BufStream::new(device);
    // We need the filesystem and file to live for the entire function.
    // Use a macro to avoid deep nesting.
    let fs;
    let session_dir;
    let mut file;

    if sd_ok {
        fs = match FileSystem::new(stream, FsOptions::new()).await {
            Ok(f) => f,
            Err(_) => {
                let _ = uart.write(b"motor_ramp: FAT FAIL\r\n").await;
                // Continue without SD
                loop { Timer::after_secs(1).await; }
            }
        };

        let mut idx: u32 = 0;
        let mut iter = fs.root_dir().iter();
        while let Some(Ok(e)) = iter.next().await {
            if e.is_dir() {
                let n = e.short_file_name_as_bytes();
                if n.len() == 7 && (n[0] == b'D' || n[0] == b'd') {
                    if let Some(i) = n[1..].iter().try_fold(0u32, |a, &b| {
                        if b >= b'0' && b <= b'9' { Some(a*10+(b-b'0') as u32) } else { None }
                    }) { idx = idx.max(i); }
                }
            }
        }
        idx += 1;
        let mut dn: String<8> = String::new();
        let _ = write!(dn, "D{:06}", idx);
        session_dir = match fs.root_dir().create_dir(dn.as_str()).await {
            Ok(d) => d,
            Err(_) => loop { Timer::after_secs(1).await; },
        };
        file = match session_dir.create_file("000001.LOG").await {
            Ok(f) => f,
            Err(_) => loop { Timer::after_secs(1).await; },
        };
        let _ = uart.write(b"motor_ramp: SD ok\r\n").await;
    } else {
        let _ = uart.write(b"motor_ramp: SD FAIL, no logging\r\n").await;
        loop { Timer::after_secs(1).await; }
    }

    // ── Motor PWM setup ──────────────────────────────────────────────────────
    let ch1 = PwmPin::<_, Ch1>::new(p.PE9,  OutputType::PushPull);
    let ch2 = PwmPin::<_, Ch2>::new(p.PE11, OutputType::PushPull);
    let ch3 = PwmPin::<_, Ch3>::new(p.PE13, OutputType::PushPull);
    let ch4 = PwmPin::<_, Ch4>::new(p.PE14, OutputType::PushPull);

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.ch1().max_duty_cycle() as u32;
    let b0 = ((max_duty * 384) >> 10) as u16;
    let b1 = ((max_duty * 768) >> 10) as u16;

    let disarm_frame = build_frame(encode(0, false), b0, b1);

    let mut dma = p.DMA1_CH1;

    // ── Phase 1: arm ESCs (5 s) ──────────────────────────────────────────────
    let _ = uart.write(b"motor_ramp: arming (5 s)...\r\n").await;
    for _ in 0u32..6100 {
        send_all(&mut pwm, &mut dma, &disarm_frame).await;
        Timer::after_micros(500).await;
    }
    led_green.set_high();

    let mut s: String<64> = String::new();
    let _ = write!(s, "motor_ramp: ramping to {} (15 s)\r\n", RAMP_MAX);
    let _ = uart.write(s.as_bytes()).await;

    // Write CSV header to SD
    file.write_all(b"t_ms,thr,ax,ay,az,gx,gy,gz\r\n").await.ok();

    let start = Instant::now();
    let mut last_log = start;
    let mut flush_counter: u16 = 0;

    // ── Phase 2: ramp + hold + ramp down ─────────────────────────────────────
    loop {
        let elapsed_ms = start.elapsed().as_millis() as u32;
        let throttle;

        if elapsed_ms < 15_000 {
            throttle = 48 + ((RAMP_MAX - 48) as u32 * elapsed_ms / 15_000) as u16;
        } else if elapsed_ms < 20_000 {
            throttle = RAMP_MAX;
        } else if elapsed_ms < 25_000 {
            let down_ms = elapsed_ms - 20_000;
            throttle = RAMP_MAX - ((RAMP_MAX - 48) as u32 * down_ms / 5_000) as u16;
        } else {
            break;
        }

        let frame = build_frame(encode(throttle, false), b0, b1);
        send_all(&mut pwm, &mut dma, &frame).await;
        Timer::after_micros(500).await;

        // Log at 50 Hz (every 20 ms)
        if last_log.elapsed().as_millis() >= 20 {
            last_log = Instant::now();
            led_blue.toggle();

            // Read IMU
            let (ax, ay, az, gx, gy, gz) = match imu.read().await {
                Ok(d) => (
                    d.accel.x as f32 * 0.004788, // 16g range: 32768 / 16 / 9.81 -> *0.004788 m/s^2
                    d.accel.y as f32 * 0.004788,
                    d.accel.z as f32 * 0.004788,
                    d.gyro.x as f32 * 0.001065,  // 2000 dps range: 32768 / 2000 * pi/180
                    d.gyro.y as f32 * 0.001065,
                    d.gyro.z as f32 * 0.001065,
                ),
                Err(_) => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            };

            let mut line: String<96> = String::new();
            let _ = write!(
                line, "{},{},{:.1},{:.1},{:.1},{:.2},{:.2},{:.2}\r\n",
                elapsed_ms, throttle, ax, ay, az, gx, gy, gz
            );
            file.write_all(line.as_bytes()).await.ok();

            flush_counter += 1;
            if flush_counter >= 25 {
                flush_counter = 0;
                file.flush().await.ok();
            }
        }
    }

    // ── Phase 3: disarm ──────────────────────────────────────────────────────
    file.flush().await.ok();
    let _ = uart.write(b"motor_ramp: disarming\r\n").await;
    led_green.set_low();
    for _ in 0u32..2000 {
        send_all(&mut pwm, &mut dma, &disarm_frame).await;
        Timer::after_micros(500).await;
    }
    let _ = uart.write(b"motor_ramp: done\r\n").await;

    loop { Timer::after_secs(1).await; }
}
