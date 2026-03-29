//! MicoAir H743 -- I2C2 bus scanner.
//!
//! Probes all 7-bit addresses on I2C2 (SCL=PB10, SDA=PB11) and prints
//! which ones respond via UART1 (PA9, 115200 baud).
//!
//! Expected devices:
//!   DPS310  barometer: 0x76 or 0x77
//!   IST8310 compass:   0x0E

#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH4, DMA1_CH5, I2C2, USART1};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<DMA1_CH0>;
    DMA1_STREAM4 => embassy_stm32::dma::InterruptHandler<DMA1_CH4>;
    DMA1_STREAM5 => embassy_stm32::dma::InterruptHandler<DMA1_CH5>;
    I2C2_EV      => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER      => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(100_000); // slow scan for reliability
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;

    let mut i2c = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, Irqs, i2c_cfg);

    let _ = uart_tx.write(b"I2C2 scan start\r\n").await;

    let mut found = 0u8;
    for addr in 0x08u8..=0x77 {
        // A zero-length write is the standard I2C scan probe.
        if i2c.write(addr, &[]).await.is_ok() {
            let mut buf: String<32> = String::new();
            write!(buf, "  found 0x{:02X}\r\n", addr).ok();
            uart_tx.write(buf.as_bytes()).await.ok();
            found += 1;
        }
        Timer::after_micros(200).await;
    }

    if found == 0 {
        let _ = uart_tx.write(b"no devices found\r\n").await;
    } else {
        let mut buf: String<32> = String::new();
        write!(buf, "scan done ({} device(s))\r\n", found).ok();
        uart_tx.write(buf.as_bytes()).await.ok();
    }

    // Blink green to show scan complete.
    loop {
        led_green.set_high();
        Timer::after_millis(500).await;
        led_green.set_low();
        Timer::after_millis(500).await;
    }
}
