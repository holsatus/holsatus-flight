//! MicoAir H743 v2 -- Spin motors one at a time at DShot300.
//!
//! Each motor is spun individually to confirm all four channels work.
//! No direction commands are sent -- ESCs use the direction stored in
//! their flash from a previous motor_dir_test run.
//!
//! Startup: disarm 3 s (all ESCs boot and arm)
//! Loop:    disarm 1 s (red) -> spin one motor 2 s (blue) -> next motor
//!
//! Channel order: Ch1=M4(PE9)=MOTOR_0, Ch2=M3(PE11)=MOTOR_1,
//!                Ch3=M2(PE13)=MOTOR_2, Ch4=M1(PE14)=MOTOR_3
//!
//! Root cause note: waveform_up_multi_channel with Ch1..Ch4 uses
//! pburst=Single in the DMA options, which means only 1 CCR is updated
//! per timer UEV instead of all 4.  The burst counter cycles
//! CCR1->CCR2->CCR3->CCR4->CCR1... so each channel's effective DShot
//! rate is 300/4 = 75 kHz.  The fix is 4 sequential single-channel
//! calls (Ch1..Ch1, Ch2..Ch2, etc.), each giving proper 300 kHz DShot.
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
use micoairh743v2::resources::MotorIrqs;
use {defmt_rtt as _, panic_probe as _};

#[path = "../config.rs"]
mod config;

// DMA1_STREAM1 (TIM1 UP DMA) is bound at lib level (resources::MotorIrqs).
bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<DMA1_CH0>;
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

/// Send one DShot frame to all four channels sequentially.
///
/// frames[0] -> Ch1 (M4, PE9)
/// frames[1] -> Ch2 (M3, PE11)
/// frames[2] -> Ch3 (M2, PE13)
/// frames[3] -> Ch4 (M1, PE14)
///
/// Each channel is sent as a single-channel DMA burst (DBL=0) at 300 kHz,
/// giving a proper 80 us DShot300 frame per channel.
async fn send_frames(
    pwm: &mut SimplePwm<'_, TIM1>,
    dma: &mut Peri<'_, DMA1_CH1>,
    frames: &[[u16; SLOTS]; 4],
) {
    let channels = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
    for (ch, frame) in channels.iter().zip(frames.iter()) {
        pwm.waveform_up_multi_channel(dma.reborrow(), MotorIrqs, *ch, *ch, frame)
            .await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_red = Output::new(p.PE3, Level::Low, Speed::Low);
    let mut led_blue = Output::new(p.PE4, Level::Low, Speed::Low);
    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart_tx =
        UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    led_green.set_high();
    Timer::after_millis(300).await;
    led_green.set_low();
    let _ = uart_tx.write(b"motor_dshot started\r\n").await;

    let ch1_pin = PwmPin::<_, Ch1>::new(p.PE9, OutputType::PushPull);
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

    let max_duty = pwm.ch1().max_duty_cycle() as u32;
    let b0 = ((max_duty * 384) >> 10) as u16;
    let b1 = ((max_duty * 768) >> 10) as u16;

    let d = build_frame(encode(0, false), b0, b1); // disarm
    let s = build_frame(encode(200, false), b0, b1); // spin

    let disarm = [d; 4];
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

    // Initial disarm 3 s.
    let _ = uart_tx.write(b"initial disarm (3 s)...\r\n").await;
    for _ in 0u16..3000 {
        send_frames(&mut pwm, &mut dma, &disarm).await;
        Timer::after_micros(500).await;
    }
    led_green.set_high();

    // Spin each motor alone for 2 s, then disarm 1 s, then next motor.
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
