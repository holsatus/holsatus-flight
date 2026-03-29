//! MicoAir H743v2 -- SPI IMU chip-ID probe.
//!
//! Reads the CHIP_ID register from every IMU CS pin on SPI2 and prints the
//! results over UART1.  Use this to confirm which IMU is physically populated.
//!
//! Expected results:
//!   BMI270  CS=PA15  -> chip_id=0x24
//!   BMI088 accel CS=PD4  -> chip_id=0x1E
//!   BMI088 gyro  CS=PD5  -> chip_id=0x0F
//!
//! 0xFF means nothing responds on that CS (MISO floating high due to Pull::Up).
//!
//! Hardware:
//!   SPI2: SCLK=PD3  MOSI=PC3  MISO=PC2
//!   UART1 TX=PA9  (115 200 baud)

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::spi::{self, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;
use heapless::String;
use micoairh743v2::resources::Spi2Irqs;
use static_cell::StaticCell;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::mode::Master;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();

/// Read register 0x00 (CHIP_ID) using the standard Bosch SPI read protocol:
/// assert CS, send [0x80, 0x00, 0x00], deassert CS, data is in byte[2].
async fn read_chip_id(dev: &mut impl SpiDevice) -> u8 {
    let mut buf = [0x80u8, 0x00, 0x00];
    dev.transfer_in_place(&mut buf).await.ok();
    buf[2]
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut uart = UartTx::new(
        p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default(),
    ).unwrap();
    uart.write(b"imu_probe: start\r\n").await.ok();

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(1_000_000); // slow for probe
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;

    let spi = Spi::new(
        p.SPI2, p.PD3, p.PC3, p.PC2,
        p.DMA1_CH6, p.DMA1_CH7,
        Spi2Irqs, spi_cfg,
    );
    let bus = SPI2_BUS.init(Mutex::new(spi));

    // Probe each CS pin; idle all others high before asserting.
    let probes: [(&str, u8); 3] = [
        ("BMI270  PA15", 0),
        ("BMI088A PD4 ", 1),
        ("BMI088G PD5 ", 2),
    ];

    // Create one device per CS, probe sequentially.
    let mut cs_pa15 = Output::new(p.PA15, Level::High, Speed::High);
    let mut cs_pd4  = Output::new(p.PD4,  Level::High, Speed::High);
    let mut cs_pd5  = Output::new(p.PD5,  Level::High, Speed::High);

    Timer::after_millis(10).await;

    for (label, idx) in probes {
        let id = match idx {
            0 => {
                let mut dev = SpiDeviceWithConfig::new(bus, &mut cs_pa15, spi_cfg);
                read_chip_id(&mut dev).await
            }
            1 => {
                let mut dev = SpiDeviceWithConfig::new(bus, &mut cs_pd4, spi_cfg);
                read_chip_id(&mut dev).await
            }
            _ => {
                let mut dev = SpiDeviceWithConfig::new(bus, &mut cs_pd5, spi_cfg);
                read_chip_id(&mut dev).await
            }
        };

        let mut s: String<48> = String::new();
        write!(s, "imu_probe: {} chip_id=0x{:02X}\r\n", label, id).ok();
        uart.write(s.as_bytes()).await.ok();

        Timer::after_millis(5).await;
    }

    uart.write(b"imu_probe: done\r\n").await.ok();
    loop { Timer::after_secs(60).await; }
}
