//! MicoAir H743 -- IST8310 compass data test.
//!
//! IST8310 on I2C2: SCL=PB10, SDA=PB11, address=0x0E (fixed).
//! UART1 (PA9 TX, 115200 baud) prints X/Y/Z counts at 10 Hz.
//! Sensitivity: 0.3 uT/LSB.

#![no_std]
#![no_main]

#[path = "../ist8310.rs"]
mod ist8310;

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH4, DMA1_CH5, I2C2, USART1};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use heapless::String;
use ist8310::Ist8310;
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

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    let mut i2c_cfg = embassy_stm32::i2c::Config::default();
    i2c_cfg.frequency = Hertz(400_000);
    i2c_cfg.scl_pullup = true;
    i2c_cfg.sda_pullup = true;

    let i2c = I2c::new(p.I2C2, p.PB10, p.PB11, p.DMA1_CH4, p.DMA1_CH5, Irqs, i2c_cfg);

    let mut mag = Ist8310::new(i2c);

    match mag.init().await {
        Ok(()) => {
            let _ = uart_tx.write(b"IST8310 ok\r\n").await;
            for _ in 0..3 {
                led_red.set_high(); led_green.set_high(); led_blue.set_high();
                Timer::after_millis(150).await;
                led_red.set_low(); led_green.set_low(); led_blue.set_low();
                Timer::after_millis(150).await;
            }
        }
        Err(e) => {
            let mut buf: String<64> = String::new();
            match e {
                ist8310::Error::I2c      => buf.push_str("IST8310: I2c error\r\n").ok(),
                ist8310::Error::WrongId(id) => {
                    write!(buf, "IST8310: wrong id=0x{:02X}\r\n", id).ok()
                }
                ist8310::Error::Timeout  => buf.push_str("IST8310: Timeout\r\n").ok(),
            };
            let _ = uart_tx.write(buf.as_bytes()).await;
            loop {
                led_red.set_high();
                Timer::after_millis(100).await;
                led_red.set_low();
                Timer::after_millis(100).await;
            }
        }
    }

    led_green.set_high();

    loop {
        match mag.read().await {
            Ok(d) => {
                let mut buf: String<64> = String::new();
                write!(buf, "mx={} my={} mz={}\r\n", d.x, d.y, d.z).ok();
                uart_tx.write(buf.as_bytes()).await.ok();

                // Indicate heading quadrant with LEDs (rough X/Y dominant axis).
                led_green.set_low(); led_blue.set_low(); led_red.set_low();
                let ax = d.x.unsigned_abs();
                let ay = d.y.unsigned_abs();
                if ax >= ay { led_blue.set_high(); } else { led_red.set_high(); }
            }
            Err(_) => {
                let _ = uart_tx.write(b"mag read err\r\n").await;
                led_red.set_high();
            }
        }
        Timer::after_millis(100).await;
    }
}
