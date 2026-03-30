//! MicoAir H743v2 -- All-sensors logger (end-to-end test).
//!
//! Reads all five sensors concurrently from a single main loop and writes
//! COBS+postcard records to five files in the same session directory on SDMMC1.
//! UART1 prints a live summary for each sensor at ~1 Hz.
//!
//! A new session directory (D%06u) is created on every run so successive runs
//! never overwrite each other.
//!
//! # Sensors and sample rates
//!   BMI088  SPI2  20 Hz  -> D000001/000001.I08
//!   BMI270  SPI3 100 Hz  -> D000001/000001.B70
//!   SPL06   I2C2   2 Hz  -> D000001/000001.BAR  (DPS310-compatible)
//!   QMC5883 I2C2  20 Hz  -> D000001/000001.MAG
//!   Battery ADC1  10 Hz  -> D000001/000001.BAT
//!
//! # Hardware
//!   BMI088 SPI2:  SCLK=PD3  MOSI=PC3  MISO=PC2  CS_ACC=PD4  CS_GYR=PD5
//!   BMI270 SPI3:  SCLK=PB3  MOSI=PD6  MISO=PB4  CS=PA15
//!   SPL06  I2C2:  SCL=PB10  SDA=PB11  addr=0x76
//!   QMC5883 I2C2: SCL=PB10  SDA=PB11  addr=0x0D
//!   Battery ADC1: PC0=voltage (1:21 divider)  PC1=current sensor
//!   SDMMC1:       CLK=PC12  CMD=PD2  D0-D3=PC8-PC11
//!   UART1:        TX=PA9  (115200 baud)
//!
//! # LEDs
//!   Green  : steady after all sensors and SD are initialised
//!   Blue   : flashes on SD write
//!   Red    : LiPo present (V > 5 V); rapid blink on init failure

#![no_std]
#![no_main]

use core::fmt::Write;

use block_device_adapters::BufStream;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::spi::{self, mode::Master as SpiMaster, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_io_async_061::Write as _;
use heapless::{String, Vec};
use postcard::to_slice_cobs;
use serde::Serialize;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::bmi088::Bmi088;
use micoairh743v2::bmi270::{Bmi270, BMI270_CONFIG_FILE};
use micoairh743v2::dps310_i2c::{self, Dps310I2c};
use micoairh743v2::qmc5883l::Qmc5883l;
use micoairh743v2::resources::{I2c2Irqs, Spi2Irqs, Spi3Irqs};
use micoairh743v2::sdlog::SdmmcResources;

// ── Log records ──────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct I08Sample { timestamp_us: u64, acc: [i16; 3], gyr: [i16; 3] }

#[derive(Serialize)]
struct B70Sample { timestamp_us: u64, acc: [i16; 3], gyr: [i16; 3] }

#[derive(Serialize)]
struct BarSample { timestamp_us: u64, pressure_pa: f32, temperature_c: f32 }

#[derive(Serialize)]
struct MagSample { timestamp_us: u64, x: i16, y: i16, z: i16 }

#[derive(Serialize)]
struct BatSample { timestamp_us: u64, voltage_mv: u32, current_mv: u32 }

// ── Interrupt bindings ───────────────────────────────────────────────────────

// I2c2Irqs, Spi2Irqs, Spi3Irqs, MotorIrqs all bound at lib level.
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

// ── Shared bus statics ───────────────────────────────────────────────────────

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;
type Spi3Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;
type I2c2Bus = Mutex<NoopRawMutex, I2c<'static, Async, Master>>;

static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();
static SPI3_BUS: StaticCell<Spi3Bus> = StaticCell::new();
static I2C2_BUS: StaticCell<I2c2Bus> = StaticCell::new();

// ADC constants.
const V_DIV:    u32 = 21;
const ADC_FULL: u32 = 65535;
const VREF_MV:  u32 = 3300;

// ── Entry point ───────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default()).unwrap();
    uart.write(b"sensors: UART ok\r\n").await.ok();

    // ── ESC silence ──────────────────────────────────────────────────────────
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());
    uart.write(b"sensors: ESC silence ok\r\n").await.ok();

    // ── SPI2 bus (BMI088) ────────────────────────────────────────────────────
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
    let spi2_bus: &'static Spi2Bus = SPI2_BUS.init(Mutex::new(spi2));

    let accel_dev = SpiDeviceWithConfig::new(spi2_bus, cs_acc, spi2_cfg);
    let gyro_dev  = SpiDeviceWithConfig::new(spi2_bus, cs_gyr, spi2_cfg);
    let mut bmi088 = Bmi088::new(accel_dev, gyro_dev);

    // ── SPI3 bus (BMI270) ────────────────────────────────────────────────────
    let cs_b70 = Output::new(p.PA15, Level::High, Speed::High);
    let spi3_init_cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(1_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = Pull::Up;
        c
    };
    let spi3 = Spi::new(p.SPI3, p.PB3, p.PD6, p.PB4, p.DMA2_CH1, p.DMA2_CH0, Spi3Irqs, spi3_init_cfg);
    let spi3_bus: &'static Spi3Bus = SPI3_BUS.init(Mutex::new(spi3));
    let b70_dev_cfg = {
        let mut c = SpiConfig::default();
        c.frequency = Hertz(10_000_000);
        c.mode = spi::MODE_3;
        c.miso_pull = Pull::Up;
        c
    };
    let b70_dev = SpiDeviceWithConfig::new(spi3_bus, cs_b70, b70_dev_cfg);
    let mut bmi270 = Bmi270::new(b70_dev);

    // ── I2C2 shared bus (SPL06 + QMC5883L) ───────────────────────────────────
    let i2c_cfg = {
        let mut c = I2cConfig::default();
        c.frequency = Hertz(400_000);
        c.scl_pullup = true;
        c.sda_pullup = true;
        c
    };
    let i2c2 = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, I2c2Irqs, i2c_cfg);
    let i2c2_bus: &'static I2c2Bus = I2C2_BUS.init(Mutex::new(i2c2));

    // Probe the SPL06 address and prime the I2C2 DMA path.
    //
    // embassy-stm32 routes empty writes through a blocking (non-DMA) path.
    // The first DMA transfer fails if no non-DMA transaction has occurred first,
    // so this probe is required even if the address is already known.
    let spl06_addr = {
        let mut bus = i2c2_bus.lock().await;
        if bus.write(dps310_i2c::ADDR_SDO_LOW, &[]).await.is_ok() {
            uart.write(b"sensors: SPL06 found at 0x76\r\n").await.ok();
            dps310_i2c::ADDR_SDO_LOW
        } else if bus.write(dps310_i2c::ADDR_SDO_HIGH, &[]).await.is_ok() {
            uart.write(b"sensors: SPL06 found at 0x77\r\n").await.ok();
            dps310_i2c::ADDR_SDO_HIGH
        } else {
            uart.write(b"sensors: SPL06 not found on 0x76 or 0x77\r\n").await.ok();
            dps310_i2c::ADDR_SDO_LOW // will fail in init, handled below
        }
    };

    let mut baro    = Dps310I2c::new(I2cDevice::new(i2c2_bus), spl06_addr);
    let mut compass = Qmc5883l::new(I2cDevice::new(i2c2_bus));

    // ── ADC1 (battery, blocking) ──────────────────────────────────────────────
    let mut adc   = Adc::new(p.ADC1);
    let mut pin_v = p.PC0;
    let mut pin_i = p.PC1;
    uart.write(b"sensors: ADC ok\r\n").await.ok();

    // ── Initialise sensors ────────────────────────────────────────────────────

    // BMI088
    for attempt in 1u8..=5 {
        match bmi088.init().await {
            Ok(()) => {
                let mut s: String<48> = String::new();
                write!(s, "sensors: BMI088 ok (attempt {})\r\n", attempt).ok();
                uart.write(s.as_bytes()).await.ok();
                break;
            }
            Err(e) if attempt < 5 => {
                use micoairh743v2::bmi088::Error as E;
                let code: u8 = match e { E::Spi => 1, E::WrongAccelId(_) => 2, E::WrongGyroId(_) => 3 };
                let mut s: String<48> = String::new();
                write!(s, "sensors: BMI088 attempt {} err={}\r\n", attempt, code).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(100).await;
            }
            Err(_) => {
                uart.write(b"sensors: BMI088 FAIL (giving up)\r\n").await.ok();
                loop {
                    led_red.set_high(); Timer::after_millis(150).await;
                    led_red.set_low();  Timer::after_millis(150).await;
                }
            }
        }
    }

    // BMI270 (config blob upload ~200 ms)
    uart.write(b"sensors: BMI270 init (~200 ms)...\r\n").await.ok();
    match bmi270.init(&BMI270_CONFIG_FILE).await {
        Ok(()) => uart.write(b"sensors: BMI270 ok\r\n").await.ok(),
        Err(e)  => {
            use micoairh743v2::bmi270::Error as E;
            let mut s: String<64> = String::new();
            match e {
                E::Spi              => s.push_str("sensors: BMI270 SPI err\r\n").ok(),
                E::WrongChipId(id)  => write!(s, "sensors: BMI270 wrong id=0x{:02X}\r\n", id).ok(),
                E::InitFailed { status } => write!(s, "sensors: BMI270 init status=0x{:02X}\r\n", status).ok(),
            };
            uart.write(s.as_bytes()).await.ok();
            loop {
                led_red.set_high(); Timer::after_millis(150).await;
                led_red.set_low();  Timer::after_millis(150).await;
            }
        }
    };

    // SPL06 / DPS310 -- retry up to 5 times.
    'baro_init: {
        for attempt in 1u8..=5 {
            match baro.init().await {
                Ok(()) => {
                    let mut s: String<48> = String::new();
                    write!(s, "sensors: SPL06 ok (attempt {})\r\n", attempt).ok();
                    uart.write(s.as_bytes()).await.ok();
                    break 'baro_init;
                }
                Err(e) => {
                    use micoairh743v2::dps310_i2c::Error as E;
                    let mut s: String<64> = String::new();
                    match e {
                        E::I2c               => write!(s, "sensors: SPL06 attempt {} I2c err\r\n", attempt).ok(),
                        E::WrongProductId(id) => write!(s, "sensors: SPL06 attempt {} wrong id=0x{:02X}\r\n", attempt, id).ok(),
                        E::NotReady          => write!(s, "sensors: SPL06 attempt {} not ready\r\n", attempt).ok(),
                        E::Timeout           => write!(s, "sensors: SPL06 attempt {} timeout\r\n", attempt).ok(),
                    };
                    uart.write(s.as_bytes()).await.ok();
                    Timer::after_millis(100).await;
                }
            }
        }
        uart.write(b"sensors: SPL06 FAIL (giving up)\r\n").await.ok();
        loop {
            led_red.set_high(); Timer::after_millis(150).await;
            led_red.set_low();  Timer::after_millis(150).await;
        }
    }

    // QMC5883L
    for attempt in 1u8..=5 {
        match compass.init().await {
            Ok(()) => {
                let mut s: String<48> = String::new();
                write!(s, "sensors: QMC5883L ok (attempt {})\r\n", attempt).ok();
                uart.write(s.as_bytes()).await.ok();
                break;
            }
            Err(_) if attempt < 5 => Timer::after_millis(100).await,
            Err(_) => {
                uart.write(b"sensors: QMC5883L FAIL (giving up)\r\n").await.ok();
                loop {
                    led_red.set_high(); Timer::after_millis(150).await;
                    led_red.set_low();  Timer::after_millis(150).await;
                }
            }
        }
    }

    // ── SDMMC1 ───────────────────────────────────────────────────────────────
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
            Ok(()) => { sd_ok = true; break; }
            Err(e) => {
                let reason = match e {
                    embassy_stm32::sdmmc::Error::NoCard          => "no card inserted",
                    embassy_stm32::sdmmc::Error::Timeout         => "card not responding",
                    embassy_stm32::sdmmc::Error::SoftwareTimeout => "software timeout",
                    embassy_stm32::sdmmc::Error::Crc             => "CRC error",
                    _                                             => "hardware error",
                };
                let mut s: String<64> = String::new();
                write!(s, "sensors: SD attempt {} FAIL: {}\r\n", attempt, reason).ok();
                uart.write(s.as_bytes()).await.ok();
                Timer::after_millis(500).await;
            }
        }
    }
    if !sd_ok {
        uart.write(b"sensors: SD FAIL (giving up)\r\n").await.ok();
        loop {
            for _ in 0..4u8 {
                led_red.set_high(); Timer::after_millis(150).await;
                led_red.set_low();  Timer::after_millis(150).await;
            }
            Timer::after_millis(800).await;
        }
    }
    uart.write(b"sensors: SD ok\r\n").await.ok();

    // ── FAT filesystem ───────────────────────────────────────────────────────
    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            uart.write(b"sensors: FAT mount FAIL\r\n").await.ok();
            loop { Timer::after_secs(1).await; }
        }
    };
    uart.write(b"sensors: FAT ok\r\n").await.ok();

    // ── Session directory (always create a new one) ───────────────────────────
    // Find the highest existing D%06u index, then keep incrementing until a
    // create_dir succeeds.  This handles the case where a previous run already
    // created the next candidate directory before crashing.
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
    let session_dir = loop {
        dir_name.clear();
        write!(dir_name, "D{:06}", session_idx).ok();
        match fs.root_dir().create_dir(dir_name.as_str()).await {
            Ok(d) => break d,
            Err(embedded_fatfs::Error::AlreadyExists) => {
                session_idx += 1;
                continue;
            }
            Err(e) => {
                let reason = match e {
                    embedded_fatfs::Error::NotEnoughSpace     => "disk full",
                    embedded_fatfs::Error::CorruptedFileSystem => "corrupted fs",
                    _                                          => "unexpected error",
                };
                let mut s: String<64> = String::new();
                write!(s, "sensors: session dir {} FAIL: {}\r\n", dir_name.as_str(), reason).ok();
                uart.write(s.as_bytes()).await.ok();
                loop { Timer::after_secs(1).await; }
            }
        }
    };

    let mut session_msg: String<32> = String::new();
    write!(session_msg, "sensors: session {}\r\n", dir_name.as_str()).ok();
    uart.write(session_msg.as_bytes()).await.ok();

    // ── Open all five log files ───────────────────────────────────────────────
    let mut file_idx: u16 = 1;

    macro_rules! open_file {
        ($ext:literal) => {{
            let mut n: String<12> = String::new();
            write!(n, "{:06}.{}", file_idx, $ext).ok();
            match session_dir.create_file(n.as_str()).await {
                Ok(f) => {
                    let mut m: String<48> = String::new();
                    write!(m, "sensors: {} open ok\r\n", n.as_str()).ok();
                    uart.write(m.as_bytes()).await.ok();
                    f
                }
                Err(_) => {
                    let mut m: String<48> = String::new();
                    write!(m, "sensors: {} create FAIL\r\n", n.as_str()).ok();
                    uart.write(m.as_bytes()).await.ok();
                    loop { Timer::after_secs(1).await; }
                }
            }
        }};
    }

    let mut file_i08 = open_file!("I08");
    let mut file_b70 = open_file!("B70");
    let mut file_bar = open_file!("BAR");
    let mut file_mag = open_file!("MAG");
    let mut file_bat = open_file!("BAT");

    uart.write(b"sensors: all files open, logging started\r\n").await.ok();
    led_green.set_high();

    // ── Per-sensor write buffers ──────────────────────────────────────────────
    let mut buf_i08: Vec<u8, 512> = Vec::new();
    let mut buf_b70: Vec<u8, 512> = Vec::new();
    let mut buf_bar: Vec<u8, 256> = Vec::new();
    let mut buf_mag: Vec<u8, 512> = Vec::new();
    let mut buf_bat: Vec<u8, 256> = Vec::new();

    let mut serde_tmp = [0u8; 32];

    let mut bytes_i08: u32 = 0;
    let mut bytes_b70: u32 = 0;
    let mut bytes_bar: u32 = 0;
    let mut bytes_mag: u32 = 0;
    let mut bytes_bat: u32 = 0;

    // ── Timing state ─────────────────────────────────────────────────────────
    let start = Instant::now();
    let mut next_i08 = start;
    let mut next_b70 = start;
    let mut next_bar = start;
    let mut next_mag = start;
    let mut next_bat = start;

    let period_i08 = Duration::from_hz(20);
    let period_b70 = Duration::from_hz(100);
    let period_bar = Duration::from_hz(2);
    let period_mag = Duration::from_hz(20);
    let period_bat = Duration::from_hz(10);

    // UART print dividers (print ~1 Hz per sensor).
    let mut div_i08: u8 = 0;
    let mut div_b70: u8 = 0;
    let mut div_bar: u8 = 0;
    let mut div_mag: u8 = 0;
    let mut div_bat: u8 = 0;

    let mut last_flush  = Instant::now();
    let mut last_rotate = Instant::now();

    const ROTATE_SECS: u64 = 30;

    // Helper: append COBS record to a Vec, flushing to file if full.
    // Declared as a closure-style macro to avoid lifetime issues with borrows.
    macro_rules! write_record {
        ($buf:expr, $file:expr, $bytes:expr, $sample:expr) => {{
            if let Ok(encoded) = to_slice_cobs(&$sample, &mut serde_tmp) {
                if $buf.extend_from_slice(encoded).is_err() {
                    led_blue.set_high();
                    if $file.write_all(&$buf).await.is_ok() {
                        $bytes += $buf.len() as u32;
                    }
                    $buf.clear();
                    led_blue.set_low();
                    $buf.extend_from_slice(encoded).ok();
                }
            }
        }};
    }

    loop {
        let now = Instant::now();

        // ── BMI270 (100 Hz) ──────────────────────────────────────────────────
        if now >= next_b70 {
            next_b70 += period_b70;
            if let Ok(d) = bmi270.read().await {
                let s = B70Sample {
                    timestamp_us: now.as_micros(),
                    acc: [d.accel.x, d.accel.y, d.accel.z],
                    gyr: [d.gyro.x,  d.gyro.y,  d.gyro.z],
                };
                write_record!(buf_b70, file_b70, bytes_b70, s);
                div_b70 = div_b70.wrapping_add(1);
                if div_b70 >= 100 {
                    div_b70 = 0;
                    let mut msg: String<96> = String::new();
                    write!(msg, "B70 ax={} ay={} az={}  gx={} gy={} gz={}\r\n",
                        d.accel.x, d.accel.y, d.accel.z,
                        d.gyro.x,  d.gyro.y,  d.gyro.z).ok();
                    uart.write(msg.as_bytes()).await.ok();
                }
            }
        }

        // ── BMI088 (20 Hz) ───────────────────────────────────────────────────
        if now >= next_i08 {
            next_i08 += period_i08;
            if let Ok(d) = bmi088.read().await {
                let s = I08Sample {
                    timestamp_us: now.as_micros(),
                    acc: [d.accel.x, d.accel.y, d.accel.z],
                    gyr: [d.gyro.x,  d.gyro.y,  d.gyro.z],
                };
                write_record!(buf_i08, file_i08, bytes_i08, s);
                div_i08 = div_i08.wrapping_add(1);
                if div_i08 >= 20 {
                    div_i08 = 0;
                    let mut msg: String<96> = String::new();
                    write!(msg, "I08 ax={} ay={} az={}  gx={} gy={} gz={}\r\n",
                        d.accel.x, d.accel.y, d.accel.z,
                        d.gyro.x,  d.gyro.y,  d.gyro.z).ok();
                    uart.write(msg.as_bytes()).await.ok();
                }
            }
        }

        // ── QMC5883L compass (20 Hz) ──────────────────────────────────────────
        if now >= next_mag {
            next_mag += period_mag;
            if let Ok(d) = compass.read().await {
                let s = MagSample {
                    timestamp_us: now.as_micros(),
                    x: d.x, y: d.y, z: d.z,
                };
                write_record!(buf_mag, file_mag, bytes_mag, s);
                div_mag = div_mag.wrapping_add(1);
                if div_mag >= 20 {
                    div_mag = 0;
                    let mut msg: String<64> = String::new();
                    write!(msg, "MAG x={} y={} z={}\r\n", d.x, d.y, d.z).ok();
                    uart.write(msg.as_bytes()).await.ok();
                }
            }
        }

        // ── Battery (10 Hz, blocking ADC) ─────────────────────────────────────
        if now >= next_bat {
            next_bat += period_bat;
            let raw_v = adc.blocking_read(&mut pin_v, SampleTime::CYCLES64_5);
            let raw_i = adc.blocking_read(&mut pin_i, SampleTime::CYCLES64_5);
            let voltage_mv = (raw_v as u32 * VREF_MV * V_DIV) / ADC_FULL;
            let current_mv = (raw_i as u32 * VREF_MV) / ADC_FULL;
            let s = BatSample { timestamp_us: now.as_micros(), voltage_mv, current_mv };
            write_record!(buf_bat, file_bat, bytes_bat, s);
            if voltage_mv > 5000 { led_red.set_high(); } else { led_red.set_low(); }
            div_bat = div_bat.wrapping_add(1);
            if div_bat >= 10 {
                div_bat = 0;
                let mut msg: String<64> = String::new();
                write!(msg, "BAT V={} mV  I_raw={} mV\r\n", voltage_mv, current_mv).ok();
                uart.write(msg.as_bytes()).await.ok();
            }
        }

        // ── SPL06 barometer (2 Hz) ────────────────────────────────────────────
        if now >= next_bar {
            next_bar += period_bar;
            if let Ok(d) = baro.read().await {
                let s = BarSample {
                    timestamp_us:  now.as_micros(),
                    pressure_pa:   d.pressure_pa,
                    temperature_c: d.temperature_c,
                };
                write_record!(buf_bar, file_bar, bytes_bar, s);
                div_bar = div_bar.wrapping_add(1);
                if div_bar >= 2 {
                    div_bar = 0;
                    let hpa  = (d.pressure_pa / 100.0) as i32;
                    let hpa_f = ((d.pressure_pa / 100.0 - hpa as f32) * 100.0).abs() as u32;
                    let tc   = d.temperature_c as i32;
                    let tc_f = ((d.temperature_c - tc as f32) * 10.0).abs() as u32;
                    let mut msg: String<64> = String::new();
                    write!(msg, "BAR p={}.{:02}hPa t={}.{}C\r\n", hpa, hpa_f, tc, tc_f).ok();
                    uart.write(msg.as_bytes()).await.ok();
                }
            }
        }

        // ── Periodic flush ────────────────────────────────────────────────────
        if last_flush.elapsed() > Duration::from_millis(500) {
            led_blue.set_high();
            macro_rules! flush_buf {
                ($buf:expr, $file:expr, $bytes:expr) => {
                    if !$buf.is_empty() {
                        if $file.write_all(&$buf).await.is_ok() { $bytes += $buf.len() as u32; }
                        $buf.clear();
                    }
                    $file.flush().await.ok();
                };
            }
            flush_buf!(buf_i08, file_i08, bytes_i08);
            flush_buf!(buf_b70, file_b70, bytes_b70);
            flush_buf!(buf_bar, file_bar, bytes_bar);
            flush_buf!(buf_mag, file_mag, bytes_mag);
            flush_buf!(buf_bat, file_bat, bytes_bat);
            led_blue.set_low();
            last_flush = Instant::now();
        }

        // ── File rotation ─────────────────────────────────────────────────────
        if last_rotate.elapsed() > Duration::from_secs(ROTATE_SECS) {
            led_blue.set_high();

            // Final flush of all buffers before closing.
            macro_rules! final_flush {
                ($buf:expr, $file:expr, $bytes:expr) => {
                    if !$buf.is_empty() {
                        if $file.write_all(&$buf).await.is_ok() { $bytes += $buf.len() as u32; }
                        $buf.clear();
                    }
                };
            }
            final_flush!(buf_i08, file_i08, bytes_i08);
            final_flush!(buf_b70, file_b70, bytes_b70);
            final_flush!(buf_bar, file_bar, bytes_bar);
            final_flush!(buf_mag, file_mag, bytes_mag);
            final_flush!(buf_bat, file_bat, bytes_bat);

            let mut rot_msg: String<96> = String::new();
            write!(rot_msg,
                "sensors: rotate {}: I08={} B70={} BAR={} MAG={} BAT={} B\r\n",
                file_idx,
                bytes_i08, bytes_b70, bytes_bar, bytes_mag, bytes_bat).ok();
            uart.write(rot_msg.as_bytes()).await.ok();

            drop(file_i08);
            drop(file_b70);
            drop(file_bar);
            drop(file_mag);
            drop(file_bat);

            bytes_i08 = 0; bytes_b70 = 0; bytes_bar = 0; bytes_mag = 0; bytes_bat = 0;
            file_idx = file_idx.wrapping_add(1);

            file_i08 = open_file!("I08");
            file_b70 = open_file!("B70");
            file_bar = open_file!("BAR");
            file_mag = open_file!("MAG");
            file_bat = open_file!("BAT");

            led_blue.set_low();
            last_rotate = Instant::now();
            last_flush  = Instant::now();
        }

        // ── Sleep until next sensor is due ────────────────────────────────────
        let mut next_wake = next_b70;
        if next_i08 < next_wake { next_wake = next_i08; }
        if next_bar < next_wake { next_wake = next_bar; }
        if next_mag < next_wake { next_wake = next_mag; }
        if next_bat < next_wake { next_wake = next_bat; }
        Timer::at(next_wake).await;
    }
}

/// Parse session index from a `D%06u` FAT 8.3 short name.
fn parse_session_dir_idx(name: &[u8]) -> Option<u32> {
    if name.len() != 7 || (name[0] != b'D' && name[0] != b'd') {
        return None;
    }
    let mut idx: u32 = 0;
    for &b in &name[1..] {
        if b < b'0' || b > b'9' { return None; }
        idx = idx * 10 + (b - b'0') as u32;
    }
    Some(idx)
}
