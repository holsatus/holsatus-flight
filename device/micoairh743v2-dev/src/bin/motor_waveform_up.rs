//! MicoAir H743 v2 -- DShot300 via waveform_up (direct CCR DMA).
//!
//! Identical test sequence to motor_dshot.rs but uses SimplePwm::waveform_up
//! instead of waveform_up_multi_channel.  waveform_up writes DMA data directly
//! to the CCRn register, bypassing the DMAR/DCR burst mechanism.
//!
//! Use this to diagnose whether Ch1/Ch4 failures are caused by the burst-mode
//! DBA routing (waveform_up_multi_channel path) or something deeper.
//!
//!   If Ch1 and Ch4 still fail  -> issue is NOT the burst DBA; check hardware.
//!   If Ch1 and Ch4 now work    -> burst DBA routing is broken for those channels.
//!
//! Channel order: Ch1=M4(PE9), Ch2=M3(PE11), Ch3=M2(PE13), Ch4=M1(PE14)
//!
//! WARNING: REMOVE PROPS before running.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH1, TIM1, USART1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::Peri;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[path = "../config.rs"]
mod config;

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<DMA1_CH0>;
    DMA1_STREAM1 => DmaInterruptHandler<DMA1_CH1>;
    USART1       => embassy_stm32::usart::InterruptHandler<USART1>;
});

const SLOTS: usize = 24;

fn encode(throttle: u16, telemetry: bool) -> u16 {
    let value = (throttle << 1) | (telemetry as u16);
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    (value << 4) | crc
}

fn build_frame(packet: u16, b0: u16, b1: u16) -> [u16; SLOTS] {
    let mut slots = [0u16; SLOTS];
    for i in 0..16 {
        let bit = (packet >> (15 - i)) & 1;
        slots[i] = if bit == 1 { b1 } else { b0 };
    }
    slots
}

/// Send one DShot frame to all four channels sequentially using waveform_up.
///
/// waveform_up writes DMA data directly to CCRn (no DMAR/DCR burst path).
///
/// frames[0] -> Ch1 (M4, PE9)
/// frames[1] -> Ch2 (M3, PE11)
/// frames[2] -> Ch3 (M2, PE13)
/// frames[3] -> Ch4 (M1, PE14)
async fn send_frames(
    pwm: &mut SimplePwm<'_, TIM1>,
    dma: &mut Peri<'_, DMA1_CH1>,
    frames: &[[u16; SLOTS]; 4],
) {
    let channels = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
    for (ch, frame) in channels.iter().zip(frames.iter()) {
        pwm.waveform_up(dma.reborrow(), Irqs, *ch, frame.as_slice()).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    led_green.set_high();
    Timer::after_millis(300).await;
    led_green.set_low();
    let _ = uart_tx.write(b"motor_waveform_up started (direct CCR DMA)\r\n").await;

    let ch1_pin = PwmPin::<_, Ch1>::new(p.PE9,  OutputType::PushPull);
    let ch2_pin = PwmPin::<_, Ch2>::new(p.PE11, OutputType::PushPull);
    let ch3_pin = PwmPin::<_, Ch3>::new(p.PE13, OutputType::PushPull);
    let ch4_pin = PwmPin::<_, Ch4>::new(p.PE14, OutputType::PushPull);

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        Some(ch2_pin),
        Some(ch3_pin),
        Some(ch4_pin),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.max_duty_cycle() as u32;
    let b0 = ((max_duty * 384) >> 10) as u16;
    let b1 = ((max_duty * 768) >> 10) as u16;

    let d = build_frame(encode(0,   false), b0, b1); // disarm
    let s = build_frame(encode(200, false), b0, b1); // spin

    let disarm  = [d; 4];
    let spin_m4 = [s, d, d, d]; // Ch1 = M4 (PE9)
    let spin_m3 = [d, s, d, d]; // Ch2 = M3 (PE11)
    let spin_m2 = [d, d, s, d]; // Ch3 = M2 (PE13)
    let spin_m1 = [d, d, d, s]; // Ch4 = M1 (PE14)

    const MOTORS: [&[u8]; 4] = [
        b"M4 Ch1 PE9\r\n",
        b"M3 Ch2 PE11\r\n",
        b"M2 Ch3 PE13\r\n",
        b"M1 Ch4 PE14\r\n",
    ];
    let spin_bufs = [&spin_m4, &spin_m3, &spin_m2, &spin_m1];

    let mut dma = p.DMA1_CH1;

    let _ = uart_tx.write(b"initial disarm (3 s)...\r\n").await;
    for _ in 0u16..3000 {
        send_frames(&mut pwm, &mut dma, &disarm).await;
        Timer::after_micros(500).await;
    }
    led_green.set_high();

    loop {
        for (label, spin_buf) in MOTORS.iter().zip(spin_bufs.iter()) {
            let _ = uart_tx.write(b"disarming (1 s)...\r\n").await;
            led_red.set_high();
            for _ in 0u16..500 {
                send_frames(&mut pwm, &mut dma, &disarm).await;
                Timer::after_micros(500).await;
            }
            led_red.set_low();

            let _ = uart_tx.write(label).await;
            led_blue.set_high();
            for _ in 0u16..1000 {
                send_frames(&mut pwm, &mut dma, spin_buf).await;
                Timer::after_micros(500).await;
            }
            led_blue.set_low();
        }
    }
}
