//! MTF-01 optical flow + lidar diagnostic tool.
//!
//! No motors. Hold the drone by hand, move it over a textured surface
//! (carpet, newspaper, wood grain) at different heights to find the
//! minimum working distance and verify axis alignment.
//!
//! Output on UART1 at 115200 baud (every 100ms):
//!   LIDAR q=<quality> d=<mm>
//!   FLOW  q=<quality> x=<raw_counts> y=<raw_counts> h=<last_lidar_m>
//!
//! Test procedure:
//!   1. Hold drone ~5cm above a textured surface -- flow should be zero/noisy
//!   2. Raise to ~15cm -- flow should respond to lateral movement
//!   3. Move drone forward -- motion_x should change
//!   4. Move drone right -- motion_y should change
//!   5. Note which axis responds to which direction for axis mapping

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartRx, UartTx};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use micoairh743v2::mtf01::{self, Frame, MAX_PAYLOAD};
use micoairh743v2::resources::{SensorIrqs, UartLogIrqs};

#[path = "../config.rs"]
mod config;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartLogIrqs, UartConfig::default()).unwrap();

    let mut mtf_cfg = UartConfig::default();
    mtf_cfg.baudrate = 115_200;
    let mut uart_mtf = UartRx::new(p.UART4, p.PA1, p.DMA1_CH3, SensorIrqs, mtf_cfg).unwrap();

    let _ = uart_tx.write(b"flow_test: MTF-01 diagnostic\r\n").await;
    let _ = uart_tx.write(b"Move drone over textured surface at various heights\r\n").await;
    let _ = uart_tx.write(b"Format: type,quality,val1,val2,height_m\r\n\r\n").await;
    led_green.set_high();

    let mut b = [0u8; 1];
    let mut hdr = [0u8; 6];
    let mut pbuf = [0u8; MAX_PAYLOAD + 1];

    let mut last_height_m: f32 = 0.0;
    let mut last_flow_q: u8 = 0;
    let mut last_flow_x: i32 = 0;
    let mut last_flow_y: i32 = 0;
    let mut last_lidar_q: u8 = 0;
    let mut last_lidar_mm: i32 = 0;
    let mut frame_count: u32 = 0;

    loop {
        // Sync to $X
        loop {
            uart_mtf.read(&mut b).await.ok();
            if b[0] != b'$' { continue; }
            uart_mtf.read(&mut b).await.ok();
            if b[0] == b'X' { break; }
        }

        uart_mtf.read(&mut hdr).await.ok();
        let size = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if size > MAX_PAYLOAD { continue; }
        uart_mtf.read(&mut pbuf[..size + 1]).await.ok();

        match mtf01::parse(hdr, &pbuf[..size + 1]) {
            Some(Frame::Lidar(l)) => {
                last_lidar_q = l.quality;
                last_lidar_mm = l.distance_mm;
                if l.distance_mm >= 0 {
                    last_height_m = l.distance_mm as f32 / 1000.0;
                }
            }
            Some(Frame::Flow(f)) => {
                last_flow_q = f.quality;
                last_flow_x = f.motion_x;
                last_flow_y = f.motion_y;
            }
            None => continue,
        }

        // Print every ~10 frames (~100ms at 100Hz combined rate)
        frame_count += 1;
        if frame_count >= 10 {
            frame_count = 0;
            led_blue.toggle();

            let mut s: String<96> = String::new();
            let _ = write!(
                s,
                "LID q={:3} d={:5}mm | FLOW q={:3} x={:+6} y={:+6} h={:.3}m\r\n",
                last_lidar_q, last_lidar_mm,
                last_flow_q, last_flow_x, last_flow_y,
                last_height_m
            );
            let _ = uart_tx.write(s.as_bytes()).await;
        }
    }
}
