//! Listen for ESC UART telemetry on UART7 RX (PE7) and echo to UART1.
//!
//! BLHeli32 / BlueJay ESCs send 10-byte telemetry frames at 115200 baud
//! when telemetry is enabled:
//!   byte 0:    temperature (deg C)
//!   byte 1-2:  voltage (10 mV, big-endian)
//!   byte 3-4:  current (10 mA, big-endian)
//!   byte 5-6:  consumption (mAh, big-endian)
//!   byte 7-8:  RPM / 100 (big-endian)
//!   byte 9:    CRC8
//!
//! Run this while all ESCs are powered (motors running or idle).  Any
//! bytes appearing on UART1 confirm the ESC is alive and sending data.
//! Silence on both DShot signal lines AND telemetry strongly suggests
//! an open/broken connection to the ESC.
//!
//! Wiring: UART7_RX = PE7 (ESC_Telemetry pad on MicoAir H743 V2)

#![no_std]
#![no_main]

use core::fmt::Write as FmtWrite;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH2, UART7, USART1};
use embassy_stm32::usart::{Config as UartConfig, UartRx, UartTx};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

#[path = "../config.rs"]
mod config;

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<DMA1_CH0>;
    DMA1_STREAM2 => DmaInterruptHandler<DMA1_CH2>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
    UART7        => embassy_stm32::usart::InterruptHandler<UART7>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);

    let mut uart1 =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    let mut cfg7 = UartConfig::default();
    cfg7.baudrate = 115_200;
    let mut uart7 = UartRx::new(p.UART7, p.PE7, p.DMA1_CH2, Irqs, cfg7).unwrap();

    led_green.set_high();
    let _ = uart1.write(b"esc_telemetry: listening on UART7/PE7 at 115200\r\n").await;
    led_green.set_low();

    let mut frame_count: u32 = 0;
    let mut buf = [0u8; 10];

    loop {
        // Read one 10-byte telemetry frame.
        match uart7.read(&mut buf).await {
            Ok(()) => {
                frame_count += 1;
                led_blue.toggle();

                let voltage_mv = ((buf[1] as u32) << 8 | buf[2] as u32) * 10;
                let rpm        = ((buf[7] as u32) << 8 | buf[8] as u32) * 100;

                let mut s: String<80> = String::new();
                let _ = write!(
                    s,
                    "#{} temp={}C volt={}mV rpm={} raw={:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}\r\n",
                    frame_count, buf[0], voltage_mv, rpm,
                    buf[0], buf[1], buf[2], buf[3], buf[4],
                    buf[5], buf[6], buf[7], buf[8], buf[9],
                );
                let _ = uart1.write(s.as_bytes()).await;
            }
            Err(_) => {
                let _ = uart1.write(b"uart7 read error\r\n").await;
            }
        }
    }
}
