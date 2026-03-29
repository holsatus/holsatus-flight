//! MicoAir H743v2 -- Battery voltage and current monitor.
//!
//! Samples ADC1 at 10 Hz and prints voltage + raw current to UART1.
//!
//! # Hardware
//!   PC0: battery voltage via 1:21 divider (V_bat = V_adc * 21)
//!   PC1: current sensor (raw mV printed; calibration depends on sensor type)
//!   UART1: TX=PA9 (115200 baud)
//!
//! # USB power (no LiPo):
//!   Voltage: reads ~0 V (divider top rail is 0 V)
//!   Current: reads ~0 mV (shunt sensor) or ~1650 mV (Hall sensor zero-current midpoint)
//!
//! # Voltage formula
//!   V_bat = (raw / 65535) * 3300 mV * 21
//!
//! # Current sensor
//!   Raw millivolts are printed to help identify the sensor type:
//!     ~0 mV at 0 A       -> shunt-based (uni-directional)
//!     ~1650 mV at 0 A    -> Hall/bi-directional (e.g. ACS712)
//!   Once identified, apply the sensor's mV/A scale factor.

#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

// DMA1_STREAM1 (TIM1 UP DMA) is bound at lib level (resources::MotorIrqs).
bind_interrupts!(struct UartIrqs {
    DMA1_STREAM0 => InterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

// Voltage divider ratio.
const V_DIV: u32 = 21;
// ADC full-scale counts (16-bit H7 ADC).
const ADC_FULL: u32 = 65535;
// Reference voltage in millivolts.
const VREF_MV: u32 = 3300;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    for _ in 0..3 {
        led_green.set_high();
        Timer::after_millis(100).await;
        led_green.set_low();
        Timer::after_millis(100).await;
    }

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, UartIrqs, UartConfig::default()).unwrap();
    uart.write(b"battery_data: UART ok\r\n").await.ok();

    // ── ESC silence ──────────────────────────────────────────────────────────
    spawner.spawn(micoairh743v2::esc_silence::task(
        p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1,
    ).unwrap());

    // ── ADC1 (blocking reads, no DMA needed at 10 Hz) ────────────────────────
    let mut adc = Adc::new(p.ADC1);
    let mut pin_v = p.PC0; // voltage (1:21 divider)
    let mut pin_i = p.PC1; // current sensor

    uart.write(b"battery_data: sampling at 10 Hz\r\n").await.ok();
    uart.write(b"battery_data: V_bat[mV]  I_raw[mV]  (0A: ~0 shunt, ~1650 Hall)\r\n").await.ok();
    led_green.set_high();

    loop {
        let raw_v = adc.blocking_read(&mut pin_v, SampleTime::CYCLES64_5);
        let raw_i = adc.blocking_read(&mut pin_i, SampleTime::CYCLES64_5);

        // Battery voltage: apply divider ratio.
        let v_bat_mv = (raw_v as u32 * VREF_MV * V_DIV) / ADC_FULL;

        // Current sensor output in raw millivolts (no scale factor yet).
        let i_raw_mv = (raw_i as u32 * VREF_MV) / ADC_FULL;

        let mut s: String<64> = String::new();
        write!(s, "V_bat={} mV  I_raw={} mV\r\n", v_bat_mv, i_raw_mv).ok();
        uart.write(s.as_bytes()).await.ok();

        // Red LED if a LiPo is present (voltage above 5 V).
        if v_bat_mv > 5000 {
            led_red.set_high();
        } else {
            led_red.set_low();
        }

        Timer::after_millis(100).await;
    }
}
