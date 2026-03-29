//! Exhaustive motor probe for MicoAir H743 V2.
//!
//! Tests each TIM1 channel (M1-M4) systematically with:
//!   1. Extended disarm (30 s) so even slow-arming ESCs can boot.
//!   2. Direction commands (like motor_dir_test) before every spin.
//!   3. Two throttle levels: 48 (minimum) and 200 (moderate).
//!   4. All-four-simultaneously test.
//!
//! Every phase is announced on UART so you can correlate sound/spin
//! with the log output.
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
use config::{Reverse, MOTOR_REVERSE_FLAGS};

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

/// Send one DShot frame to all four channels via sequential single-channel bursts.
/// frames[0..3] map to Ch1..Ch4.
async fn send4(
    pwm: &mut SimplePwm<'_, TIM1>,
    dma: &mut Peri<'_, DMA1_CH1>,
    frames: &[[u16; SLOTS]; 4],
) {
    let channels = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
    for (ch, frame) in channels.iter().zip(frames.iter()) {
        pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, *ch, *ch, frame).await;
    }
}

/// Repeat send4 for `count` frames with a 500 us inter-frame gap.
async fn burst4(
    pwm: &mut SimplePwm<'_, TIM1>,
    dma: &mut Peri<'_, DMA1_CH1>,
    frames: &[[u16; SLOTS]; 4],
    count: u32,
) {
    for _ in 0..count {
        send4(pwm, dma, frames).await;
        Timer::after_micros(500).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue  = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    led_green.set_high();
    Timer::after_millis(500).await;
    led_green.set_low();
    let _ = uart.write(b"motor_probe started\r\n").await;

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::<_, Ch1>::new(p.PE9,  OutputType::PushPull)),
        Some(PwmPin::<_, Ch2>::new(p.PE11, OutputType::PushPull)),
        Some(PwmPin::<_, Ch3>::new(p.PE13, OutputType::PushPull)),
        Some(PwmPin::<_, Ch4>::new(p.PE14, OutputType::PushPull)),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max = pwm.ch1().max_duty_cycle() as u32;
    let b0 = ((max * 384) >> 10) as u16;
    let b1 = ((max * 768) >> 10) as u16;

    let d = build_frame(encode(0,   false), b0, b1); // disarm / zero throttle
    let s = build_frame(encode(200, false), b0, b1); // moderate throttle
    let lo = build_frame(encode(48,  false), b0, b1); // minimum non-zero throttle

    // Direction command frames (per MOTOR_REVERSE_FLAGS).
    let rev = [
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_0)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_1)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_2)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_3)), b0, b1),
    ];

    let all_disarm = [d; 4];
    let all_rev    = rev;
    let all_spin   = [s; 4];

    let mut dma = p.DMA1_CH1;

    // ----------------------------------------------------------------
    // Phase 0: Extended disarm -- 30 s so all ESCs can boot and arm.
    // ----------------------------------------------------------------
    let _ = uart.write(b"\r\n[phase 0] extended disarm 30 s -- wait for all ESC beeps\r\n").await;
    led_red.set_high();
    // ~1820 frames/s (each send4 ~320us + 500us gap = ~820us).
    // 30 s / 820us = ~36585 iterations.
    for _ in 0u32..36000 {
        send4(&mut pwm, &mut dma, &all_disarm).await;
        Timer::after_micros(500).await;
    }
    led_red.set_low();
    let _ = uart.write(b"[phase 0] done\r\n").await;

    // ----------------------------------------------------------------
    // Phase 1: Direction commands -- store correct spin direction in
    // each ESC's flash.  Some ESCs reboot here; the 3 s post-dir
    // disarm lets them re-arm.
    // ----------------------------------------------------------------
    let _ = uart.write(b"[phase 1] sending direction commands (30x) ...\r\n").await;
    for _ in 0..30 {
        send4(&mut pwm, &mut dma, &all_rev).await;
        Timer::after_millis(1).await;
    }

    let _ = uart.write(b"[phase 1] post-direction disarm 5 s\r\n").await;
    for _ in 0u32..6000 {
        send4(&mut pwm, &mut dma, &all_disarm).await;
        Timer::after_micros(500).await;
    }
    let _ = uart.write(b"[phase 1] done\r\n").await;

    // ----------------------------------------------------------------
    // Phase 2: All four simultaneously at throttle 200 for 10 s.
    // ----------------------------------------------------------------
    let _ = uart.write(b"[phase 2] all 4 motors: throttle 200 for 10 s\r\n").await;
    led_blue.set_high();
    burst4(&mut pwm, &mut dma, &all_spin, 12000).await;
    led_blue.set_low();

    let _ = uart.write(b"[phase 2] disarm 3 s\r\n").await;
    burst4(&mut pwm, &mut dma, &all_disarm, 3600).await;

    // ----------------------------------------------------------------
    // Phase 3: Per-motor at throttle 200 (10 s each).
    // Ch1=M4, Ch2=M3, Ch3=M2, Ch4=M1.
    // ----------------------------------------------------------------
    const LABELS_200: [&[u8]; 4] = [
        b"[phase 3] M4 Ch1 PE9  thr=200 10s\r\n",
        b"[phase 3] M3 Ch2 PE11 thr=200 10s\r\n",
        b"[phase 3] M2 Ch3 PE13 thr=200 10s\r\n",
        b"[phase 3] M1 Ch4 PE14 thr=200 10s\r\n",
    ];

    for (i, label) in LABELS_200.iter().enumerate() {
        let _ = uart.write(b"[phase 3] disarm 5 s\r\n").await;
        burst4(&mut pwm, &mut dma, &all_disarm, 6000).await;

        let _ = uart.write(label).await;
        led_blue.set_high();
        let mut spin_buf = [d; 4];
        spin_buf[i] = s;
        burst4(&mut pwm, &mut dma, &spin_buf, 12000).await;
        led_blue.set_low();
    }

    let _ = uart.write(b"[phase 3] disarm 5 s\r\n").await;
    burst4(&mut pwm, &mut dma, &all_disarm, 6000).await;

    // ----------------------------------------------------------------
    // Phase 4: Per-motor at throttle 48 (minimum, 10 s each).
    // ----------------------------------------------------------------
    const LABELS_48: [&[u8]; 4] = [
        b"[phase 4] M4 Ch1 PE9  thr=48  10s\r\n",
        b"[phase 4] M3 Ch2 PE11 thr=48  10s\r\n",
        b"[phase 4] M2 Ch3 PE13 thr=48  10s\r\n",
        b"[phase 4] M1 Ch4 PE14 thr=48  10s\r\n",
    ];

    for (i, label) in LABELS_48.iter().enumerate() {
        let _ = uart.write(b"[phase 4] disarm 5 s\r\n").await;
        burst4(&mut pwm, &mut dma, &all_disarm, 6000).await;

        let _ = uart.write(label).await;
        led_blue.set_high();
        let mut spin_buf = [d; 4];
        spin_buf[i] = lo;
        burst4(&mut pwm, &mut dma, &spin_buf, 12000).await;
        led_blue.set_low();
    }

    // ----------------------------------------------------------------
    // Phase 5: Repeat indefinitely -- all 4 simultaneously.
    // This gives an ongoing baseline to compare against.
    // ----------------------------------------------------------------
    let _ = uart.write(b"[phase 5] all 4 motors simultaneously, loop\r\n").await;
    loop {
        let _ = uart.write(b"[phase 5] disarm 3 s\r\n").await;
        led_red.set_high();
        burst4(&mut pwm, &mut dma, &all_disarm, 3600).await;
        led_red.set_low();

        let _ = uart.write(b"[phase 5] spin all thr=200 5 s\r\n").await;
        led_blue.set_high();
        burst4(&mut pwm, &mut dma, &all_spin, 6000).await;
        led_blue.set_low();
    }
}
