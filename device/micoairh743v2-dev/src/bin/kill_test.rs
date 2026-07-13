//! Kill switch test -- no motors, just IMU + flip detection.
//!
//! Uses the EXACT same thresholds as flight.rs flip_kill task.
//! Hold the drone and tilt/flip it by hand to verify the kill
//! triggers at the right angle.
//!
//! UART prints at 20 Hz with status. Red LED = warning/kill.

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::spi::{self, mode::Master as SpiMaster, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_stm32::mode::Async;
use embassy_time::Timer;
use heapless::String;
use static_cell::StaticCell;
use micoairh743v2::bmi088::Bmi088;
use micoairh743v2::resources::Spi2Irqs;
use micoairh743v2::resources::UartLogIrqs;
use {defmt_rtt as _, panic_probe as _};

#[path = "../config.rs"]
mod config;


// ── Same constants as flight.rs flip_kill ──
const AZ_INVERTED_THRESHOLD: f32 = -3.0;
const FLIP_COUNT_THRESHOLD: u32 = 10;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default()).unwrap();

    let cs_acc = Output::new(p.PD4, Level::High, Speed::High);
    let cs_gyr = Output::new(p.PD5, Level::High, Speed::High);
    let spi2_cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(8_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = Pull::Up;
        c
    };
    let spi2 = Spi::new(p.SPI2, p.PD3, p.PC3, p.PC2, p.DMA1_CH6, p.DMA1_CH7, Spi2Irqs, spi2_cfg);
    type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;
    static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();
    let spi2_bus: &'static Spi2Bus = SPI2_BUS.init(Mutex::new(spi2));
    let accel_dev = SpiDeviceWithConfig::new(spi2_bus, cs_acc, spi2_cfg);
    let gyro_dev  = SpiDeviceWithConfig::new(spi2_bus, cs_gyr, spi2_cfg);
    let mut imu = Bmi088::new(accel_dev, gyro_dev);

    let _ = uart.write(b"kill_test: initializing IMU...\r\n").await;
    for attempt in 1u8..=5 {
        match imu.init().await {
            Ok(()) => { let _ = uart.write(b"kill_test: BMI088 OK\r\n").await; break; }
            Err(_) if attempt < 5 => Timer::after_millis(100).await,
            Err(_) => {
                let _ = uart.write(b"kill_test: BMI088 FAIL\r\n").await;
                loop { Timer::after_secs(1).await; }
            }
        }
    }

    let _ = uart.write(b"\r\nkill_test: tilt drone to test flip detection\r\n").await;
    let _ = uart.write(b"  az > 0   = OK (upright)\r\n").await;
    let _ = uart.write(b"  az < -3  = KILL (cnt=1, instant)\r\n").await;
    let _ = uart.write(b"\r\n").await;
    led_green.set_high();

    let mut inverted_count: u32 = 0;
    let mut print_div: u8 = 0;

    loop {
        if let Ok(d) = imu.read().await {
            // BMI088 accel Z -- same conversion as flight.rs imu_reader
            let az = d.accel.z as f32 * (9.81 / 1024.0);

            // ── Exact same logic as flight.rs flip_kill ──
            if az < AZ_INVERTED_THRESHOLD {
                inverted_count += 1;
                if inverted_count >= FLIP_COUNT_THRESHOLD {
                    led_red.set_high();
                    led_green.set_low();
                    let _ = uart.write(b"\r\n[kill] FLIP DETECTED -- motors would be cut\r\n").await;
                    for _ in 0..10 {
                        let _ = uart.write(b"[kill] *** MOTORS OFF ***\r\n").await;
                        led_red.toggle();
                        Timer::after_millis(200).await;
                    }
                    let _ = uart.write(b"[kill] power cycle to reset\r\n").await;
                    loop {
                        led_red.toggle();
                        Timer::after_millis(500).await;
                    }
                }
            } else {
                inverted_count = 0;
            }

            // Print at ~20 Hz (IMU reads at ~1kHz, print every 50th)
            print_div = print_div.wrapping_add(1);
            if print_div >= 50 {
                print_div = 0;
                let status = if az < AZ_INVERTED_THRESHOLD {
                    led_red.set_high();
                    "INVERTED"
                } else if az < 3.0 {
                    led_red.set_high();
                    "TILTED"
                } else {
                    led_red.set_low();
                    "OK"
                };
                let mut s: String<48> = String::new();
                let _ = write!(s, "az={:.1} {}\r\n", az, status);
                let _ = uart.write(s.as_bytes()).await;
            }
        }

        Timer::after_millis(1).await; // ~1kHz IMU read rate
    }
}
