//! MicoAir H743 -- Motor analog PWM test.
//!
//! Standard 50 Hz servo PWM (1000-2000 us) on all 4 motor pins.
//! Uses embassy Timer for accurate pulse width - no calibration needed.
//!
//! M1 -> PE14 (TIM1 CH4)
//! M2 -> PE13 (TIM1 CH3)
//! M3 -> PE11 (TIM1 CH2)
//! M4 -> PE9  (TIM1 CH1)
//!
//! Pulse width: 1000 us = min/disarm, 2000 us = max throttle.
//!
//! WARNING: REMOVE PROPS before running.

#![no_std]
#![no_main]

use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, USART1};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<DMA1_CH0>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

/// Send one 50 Hz PWM pulse to all four motors.
/// pulse_us: HIGH time in microseconds (1000 = min, 2000 = max).
/// Total period is 20 ms.
async fn send_pwm(
    m1: &mut Output<'_>, m2: &mut Output<'_>,
    m3: &mut Output<'_>, m4: &mut Output<'_>,
    pulse_us: u64,
) {
    m1.set_high(); m2.set_high(); m3.set_high(); m4.set_high();
    Timer::after_micros(pulse_us).await;
    m1.set_low(); m2.set_low(); m3.set_low(); m4.set_low();
    Timer::after_micros(20_000 - pulse_us).await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    let mut m1 = Output::new(p.PE14, Level::Low, Speed::VeryHigh);
    let mut m2 = Output::new(p.PE13, Level::Low, Speed::VeryHigh);
    let mut m3 = Output::new(p.PE11, Level::Low, Speed::VeryHigh);
    let mut m4 = Output::new(p.PE9,  Level::Low, Speed::VeryHigh);

    led_green.set_high();
    Timer::after_millis(300).await;
    led_green.set_low();
    let _ = uart_tx.write(b"motor_pwm started (analog PWM 50 Hz)\r\n").await;
    defmt::info!("motor_pwm started (analog PWM 50 Hz)");

    led_green.set_high();

    loop {
        // Arm: send 1000 us pulse at 50 Hz for 5 s.
        let _ = uart_tx.write(b"Arming ESCs (5 s, 1000 us)...\r\n").await;
        led_red.set_high();
        for _ in 0u16..250 {
            send_pwm(&mut m1, &mut m2, &mut m3, &mut m4, 1000).await;
        }
        led_red.set_low();

        // Ramp from 1100 us to 1400 us over 3 s.
        let _ = uart_tx.write(b"Spinning (ramp 1100->1400 us)\r\n").await;
        led_blue.set_high();
        for step in 0u16..150 {
            let pulse_us = 1100 + (step * 2) as u64;
            send_pwm(&mut m1, &mut m2, &mut m3, &mut m4, pulse_us).await;
        }
        // Hold at 1400 us for 2 s.
        let _ = uart_tx.write(b"Holding 1400 us\r\n").await;
        for _ in 0u16..100 {
            send_pwm(&mut m1, &mut m2, &mut m3, &mut m4, 1400).await;
        }
        led_blue.set_low();

        let _ = uart_tx.write(b"cycle done - pause 2 s\r\n").await;
        for _ in 0u16..100 {
            send_pwm(&mut m1, &mut m2, &mut m3, &mut m4, 1000).await;
        }
    }
}
