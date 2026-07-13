//! BMI088 accelerometer + gyroscope bias calibration.
//!
//! Place the drone perfectly level (props against underside of a level table),
//! connect power, and wait. The binary samples 2000 readings (~2 seconds),
//! computes the mean, subtracts expected gravity from az, and prints the
//! bias values to UART1 every 3 seconds (repeated so you can read at leisure).
//!
//! Copy the printed bias values into config.rs BMI088_ACC_BIAS.
//!
//! No motors spin. No SD card access. Just IMU + UART.
//!
//! Usage:
//!   make flash-release BIN=imu_cal
//!   # open miniterm on UART1 at 115200

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{self, mode::Master, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use heapless::String;
use micoairh743v2::bmi088::Bmi088;
use micoairh743v2::resources::Spi2Irqs;
use micoairh743v2::resources::UartLogIrqs;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};


type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();

const ACC_SCALE: f32 = 9.81 / 5460.8;
const GYR_SCALE: f32 = core::f32::consts::PI / (180.0 * 16.384);
const GRAVITY: f32 = 9.80665;
const N: u32 = 2000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led = Output::new(p.PE2, Level::High, Speed::Low);

    let mut uart = UartTx::new(
        p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default(),
    ).unwrap();

    uart.write(b"\r\nimu_cal: place drone LEVEL and hold still\r\n").await.ok();
    uart.write(b"imu_cal: sampling 2000 readings...\r\n").await.ok();

    // SPI2 setup (same as imu_probe / imu_data)
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(8_000_000);
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;

    let spi = Spi::new(
        p.SPI2, p.PD3, p.PC3, p.PC2,
        p.DMA1_CH6, p.DMA1_CH7,
        Spi2Irqs, spi_cfg,
    );
    let bus = SPI2_BUS.init(Mutex::new(spi));

    let mut cs_acc = Output::new(p.PD4, Level::High, Speed::High);
    let mut cs_gyr = Output::new(p.PD5, Level::High, Speed::High);

    let accel_dev = SpiDeviceWithConfig::new(bus, &mut cs_acc, spi_cfg);
    let gyro_dev = SpiDeviceWithConfig::new(bus, &mut cs_gyr, spi_cfg);

    let mut imu = Bmi088::new(accel_dev, gyro_dev);

    match imu.init().await {
        Ok(()) => { uart.write(b"imu_cal: BMI088 init OK\r\n").await.ok(); }
        Err(_) => {
            uart.write(b"imu_cal: BMI088 init FAILED\r\n").await.ok();
            loop { Timer::after_secs(60).await; }
        }
    }

    // Wait 500ms for sensor to settle after init
    Timer::after_millis(500).await;

    // Accumulate samples
    let mut sum_acc = [0.0_f64; 3];
    let mut sum_gyr = [0.0_f64; 3];

    for i in 0..N {
        match imu.read().await {
            Ok(d) => {
                sum_acc[0] += (d.accel.x as f32 * ACC_SCALE) as f64;
                sum_acc[1] += (d.accel.y as f32 * ACC_SCALE) as f64;
                sum_acc[2] += (d.accel.z as f32 * ACC_SCALE) as f64;
                sum_gyr[0] += (d.gyro.x as f32 * GYR_SCALE) as f64;
                sum_gyr[1] += (d.gyro.y as f32 * GYR_SCALE) as f64;
                sum_gyr[2] += (d.gyro.z as f32 * GYR_SCALE) as f64;
            }
            Err(_) => {
                let mut s: String<48> = String::new();
                write!(s, "imu_cal: read error at sample {}\r\n", i).ok();
                uart.write(s.as_bytes()).await.ok();
            }
        }
        Timer::after_millis(1).await; // ~1 kHz sampling
    }

    let n = N as f64;
    let acc_bias = [
        (sum_acc[0] / n) as f32,
        (sum_acc[1] / n) as f32,
        (sum_acc[2] / n) as f32 - GRAVITY, // subtract expected +g
    ];
    let gyr_bias = [
        (sum_gyr[0] / n) as f32,
        (sum_gyr[1] / n) as f32,
        (sum_gyr[2] / n) as f32,
    ];

    // Print results in a loop so user can read at leisure
    loop {
        led.toggle();

        let mut s: String<96> = String::new();
        write!(
            s, "ACC bias: [{:.4}, {:.4}, {:.4}]\r\n",
            acc_bias[0], acc_bias[1], acc_bias[2]
        ).ok();
        uart.write(s.as_bytes()).await.ok();

        let mut s: String<96> = String::new();
        write!(
            s, "GYR bias: [{:.4}, {:.4}, {:.4}]\r\n",
            gyr_bias[0], gyr_bias[1], gyr_bias[2]
        ).ok();
        uart.write(s.as_bytes()).await.ok();

        let mut s: String<96> = String::new();
        write!(
            s, "ACC mean: [{:.4}, {:.4}, {:.4}]\r\n",
            acc_bias[0], acc_bias[1], acc_bias[2] + GRAVITY
        ).ok();
        uart.write(s.as_bytes()).await.ok();

        uart.write(b"---\r\n").await.ok();
        uart.write(b"Copy ACC bias to config.rs BMI088_ACC_BIAS\r\n\r\n").await.ok();

        Timer::after_secs(3).await;
    }
}
