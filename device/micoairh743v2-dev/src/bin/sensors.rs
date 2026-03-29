//! MicoAir H743 -- BMI270 + DPS310 + IST8310 parallel sensor test.
//!
//! BMI270 on SPI2:  MOSI=PC3, MISO=PC2, SCLK=PD3, CS=PA15
//! DPS310 on I2C2:  SCL=PB10, SDA=PB11, addr=0x76
//! IST8310 on I2C2: SCL=PB10, SDA=PB11, addr=0x0E
//!
//! Three tasks run concurrently sharing the buses through Mutex wrappers:
//!   run_imu:     BMI270 at 20 Hz, signals LED task
//!   run_baro:    DPS310 at 2 Hz
//!   run_compass: IST8310 at 10 Hz
//!
//! All UART writes serialised through a CriticalSection Mutex.

#![no_std]
#![no_main]

#[path = "../bmi270.rs"]
mod bmi270;
#[path = "../dps310_i2c.rs"]
mod dps310_i2c;
#[path = "../ist8310.rs"]
mod ist8310;

use bmi270::{Bmi270, ImuData};
use core::fmt::Write;
use dps310_i2c::Dps310I2c;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, I2C2, USART1};
use embassy_stm32::rcc::mux::Saisel;
use embassy_stm32::spi::{self, mode::Master as SpiMaster, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use heapless::String;
use ist8310::Ist8310;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<DMA1_CH0>;
    DMA1_STREAM4 => embassy_stm32::dma::InterruptHandler<DMA1_CH4>;
    DMA1_STREAM5 => embassy_stm32::dma::InterruptHandler<DMA1_CH5>;
    DMA1_STREAM6 => embassy_stm32::dma::InterruptHandler<DMA1_CH6>;
    DMA1_STREAM7 => embassy_stm32::dma::InterruptHandler<DMA1_CH7>;
    I2C2_EV      => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER      => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;
type I2c2Bus = Mutex<NoopRawMutex, I2c<'static, Async, Master>>;
type UartMtx = Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>;

static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();
static I2C2_BUS: StaticCell<I2c2Bus> = StaticCell::new();
static UART_MTX: StaticCell<UartMtx> = StaticCell::new();

static IMU_SIGNAL: Signal<CriticalSectionRawMutex, ImuData> = Signal::new();

// ── LED task ─────────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn led_task(
    mut led_green: Output<'static>,
    mut led_blue:  Output<'static>,
    mut led_red:   Output<'static>,
) {
    let first = embassy_time::with_timeout(
        embassy_time::Duration::from_millis(5000),
        IMU_SIGNAL.wait(),
    ).await;

    if first.is_err() {
        loop {
            led_red.set_high(); led_blue.set_low();
            Timer::after_millis(100).await;
            led_red.set_low(); led_blue.set_high();
            Timer::after_millis(100).await;
        }
    }

    loop {
        let d = IMU_SIGNAL.wait().await;
        let ax = d.accel.x.unsigned_abs();
        let ay = d.accel.y.unsigned_abs();
        let az = d.accel.z.unsigned_abs();
        led_green.set_low(); led_blue.set_low(); led_red.set_low();
        if az >= ax && az >= ay { led_green.set_high(); }
        else if ax >= ay        { led_blue.set_high(); }
        else                    { led_red.set_high(); }
    }
}

// ── IMU task ──────────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn run_imu(bus: &'static Spi2Bus, cs: Output<'static>, uart: &'static UartMtx) {
    let cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(8_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = Pull::Up;
        c
    };
    let dev = SpiDeviceWithConfig::new(bus, cs, cfg);
    let mut imu = Bmi270::new(dev);

    for attempt in 0..5u8 {
        match imu.init(&bmi270::BMI270_CONFIG_FILE).await {
            Ok(()) => {
                let mut buf: String<64> = String::new();
                write!(buf, "BMI270 ok (attempt {})\r\n", attempt + 1).ok();
                uart.lock().await.write(buf.as_bytes()).await.ok();
                break;
            }
            Err(e) => {
                let code: u8 = match e {
                    bmi270::Error::Spi => 1,
                    bmi270::Error::WrongChipId(_) => 2,
                    bmi270::Error::InitFailed { .. } => 3,
                };
                let mut buf: String<64> = String::new();
                write!(buf, "BMI270 attempt {} err={}\r\n", attempt + 1, code).ok();
                uart.lock().await.write(buf.as_bytes()).await.ok();
                Timer::after_millis(100).await;
            }
        }
    }

    loop {
        if let Ok(d) = imu.read().await {
            IMU_SIGNAL.signal(d);
            let mut buf: String<96> = String::new();
            write!(buf, "IMU ax={} ay={} az={} gx={} gy={} gz={}\r\n",
                d.accel.x, d.accel.y, d.accel.z,
                d.gyro.x, d.gyro.y, d.gyro.z).ok();
            uart.lock().await.write(buf.as_bytes()).await.ok();
        }
        Timer::after_millis(50).await;
    }
}

// ── Baro task ─────────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn run_baro(bus: &'static I2c2Bus, uart: &'static UartMtx) {
    let dev = I2cDevice::new(bus);
    let mut baro = Dps310I2c::new(dev, dps310_i2c::ADDR_SDO_LOW);

    match baro.init().await {
        Ok(()) => {
            uart.lock().await.write(b"DPS310 ok\r\n").await.ok();
        }
        Err(e) => {
            let mut buf: String<64> = String::new();
            match e {
                dps310_i2c::Error::I2c              => buf.push_str("DPS310: I2c err\r\n").ok(),
                dps310_i2c::Error::WrongProductId(id) => {
                    write!(buf, "DPS310: wrong id=0x{:02X}\r\n", id).ok()
                }
                dps310_i2c::Error::NotReady => buf.push_str("DPS310: NotReady\r\n").ok(),
                dps310_i2c::Error::Timeout  => buf.push_str("DPS310: Timeout\r\n").ok(),
            };
            uart.lock().await.write(buf.as_bytes()).await.ok();
            loop { Timer::after_millis(1000).await; }
        }
    }

    loop {
        if let Ok(d) = baro.read().await {
            let press_hpa  = (d.pressure_pa / 100.0) as i32;
            let press_frac = ((d.pressure_pa / 100.0 - press_hpa as f32) * 100.0).abs() as u32;
            let temp_i     = d.temperature_c as i32;
            let temp_frac  = ((d.temperature_c - temp_i as f32) * 10.0).abs() as u32;
            let mut buf: String<64> = String::new();
            write!(buf, "BARO p={}.{:02}hPa t={}.{}C\r\n",
                press_hpa, press_frac, temp_i, temp_frac).ok();
            uart.lock().await.write(buf.as_bytes()).await.ok();
        }
        Timer::after_millis(500).await;
    }
}

// ── Compass task ──────────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn run_compass(bus: &'static I2c2Bus, uart: &'static UartMtx) {
    let dev = I2cDevice::new(bus);
    let mut mag = Ist8310::new(dev);

    match mag.init().await {
        Ok(()) => {
            uart.lock().await.write(b"IST8310 ok\r\n").await.ok();
        }
        Err(e) => {
            let mut buf: String<64> = String::new();
            match e {
                ist8310::Error::I2c         => buf.push_str("IST8310: I2c err\r\n").ok(),
                ist8310::Error::WrongId(id) => {
                    write!(buf, "IST8310: wrong id=0x{:02X}\r\n", id).ok()
                }
                ist8310::Error::Timeout => buf.push_str("IST8310: Timeout\r\n").ok(),
            };
            uart.lock().await.write(buf.as_bytes()).await.ok();
            loop { Timer::after_millis(1000).await; }
        }
    }

    loop {
        if let Ok(d) = mag.read().await {
            let mut buf: String<64> = String::new();
            write!(buf, "MAG mx={} my={} mz={}\r\n", d.x, d.y, d.z).ok();
            uart.lock().await.write(buf.as_bytes()).await.ok();
        }
        Timer::after_millis(100).await;
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux.spi123sel = Saisel::PER;
    let p = embassy_stm32::init(config);

    let led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let uart_tx = UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();
    let uart: &'static UartMtx = UART_MTX.init(Mutex::new(uart_tx));

    uart.lock().await.write(b"sensors started\r\n").await.ok();

    // SPI2 bus: CS must be HIGH before Spi::new().
    let imu_cs = Output::new(p.PA15, Level::High, Speed::High);
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(8_000_000);
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;
    let spi = Spi::new(p.SPI2, p.PD3, p.PC3, p.PC2, p.DMA1_CH6, p.DMA1_CH7, Irqs, spi_cfg);
    let spi_bus: &'static Spi2Bus = SPI2_BUS.init(Mutex::new(spi));

    // I2C2 bus: shared between DPS310 and IST8310.
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;
    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, Irqs, i2c_cfg);
    let i2c_bus: &'static I2c2Bus = I2C2_BUS.init(Mutex::new(i2c));

    spawner.spawn(led_task(led_green, led_blue, led_red).expect("led_task"));
    spawner.spawn(run_imu(spi_bus, imu_cs, uart).expect("run_imu"));
    spawner.spawn(run_baro(i2c_bus, uart).expect("run_baro"));
    spawner.spawn(run_compass(i2c_bus, uart).expect("run_compass"));
}
