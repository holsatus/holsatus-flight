//! DShot integration test: embassy_config() + P10 interrupt executor.
//!
//! Identical motor sequence to motor_dshot.rs (3000 disarm + direction +
//! 200 disarm + 2000 spin at throttle=200, looping forever) but with two
//! differences from that binary:
//!
//!   1. Uses embassy_config() (PLL1, 400 MHz SYSCLK, TIM1 at 200 MHz)
//!      instead of Default::default() (HSI, 64 MHz).
//!   2. The DShot sequence runs inside a FDCAN1_IT0 interrupt executor
//!      at P10 priority, same as motor_governor_task in flight.rs.
//!
//! If motors spin here: the hardware layer (clock + interrupt executor +
//! direct waveform calls) is fine. The issue is elsewhere in flight.rs
//! (DshotDriver abstraction, motor_governor timing, or task interactions).
//!
//! If motors do NOT spin here: the clock config or the interrupt executor
//! itself is the problem.
//!
//! WARNING: remove props before running.

#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::{bind_interrupts, interrupt, interrupt::{InterruptExt, Priority}, peripherals, Peri};
use embassy_stm32::dma::InterruptHandler as DmaInterruptHandler;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Pull in config without linking the full micoairh743 lib (avoids duplicate ISR).
#[path = "../config.rs"]
mod config;
use config::{Reverse, MOTOR_REVERSE_FLAGS};

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<peripherals::DMA1_CH0>;
    DMA1_STREAM1 => DmaInterruptHandler<peripherals::DMA1_CH1>;
    USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
});

const FRAME_SLOTS: usize = 24;
const N_CHAN: usize = 4;

fn encode(throttle: u16, telemetry: bool) -> u16 {
    let value = (throttle << 1) | (telemetry as u16);
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    (value << 4) | crc
}

fn build_frame(packet: u16, b0: u16, b1: u16) -> [u16; FRAME_SLOTS] {
    let mut slots = [0u16; FRAME_SLOTS];
    for i in 0..16 {
        let bit = (packet >> (15 - i)) & 1;
        slots[i] = if bit == 1 { b1 } else { b0 };
    }
    slots
}

fn build_interleaved(frames: [[u16; FRAME_SLOTS]; N_CHAN]) -> [u16; FRAME_SLOTS * N_CHAN] {
    let mut buf = [0u16; FRAME_SLOTS * N_CHAN];
    for t in 0..FRAME_SLOTS {
        buf[t * N_CHAN + 0] = frames[0][t];
        buf[t * N_CHAN + 1] = frames[1][t];
        buf[t * N_CHAN + 2] = frames[2][t];
        buf[t * N_CHAN + 3] = frames[3][t];
    }
    buf
}

fn bit_duties(max_duty: u32) -> (u16, u16) {
    let b0 = ((max_duty * 384) >> 10) as u16;
    let b1 = ((max_duty * 768) >> 10) as u16;
    (b0, b1)
}

static EXECUTOR_P10: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn FDCAN1_IT0() {
    EXECUTOR_P10.on_interrupt()
}

#[embassy_executor::task]
async fn motor_task(
    tim: Peri<'static, peripherals::TIM1>,
    m4:  Peri<'static, peripherals::PE9>,
    m3:  Peri<'static, peripherals::PE11>,
    m2:  Peri<'static, peripherals::PE13>,
    m1:  Peri<'static, peripherals::PE14>,
    mut dma: Peri<'static, peripherals::DMA1_CH1>,
) -> ! {
    let ch1 = PwmPin::<_, Ch1>::new(m4, OutputType::PushPull);
    let ch2 = PwmPin::<_, Ch2>::new(m3, OutputType::PushPull);
    let ch3 = PwmPin::<_, Ch3>::new(m2, OutputType::PushPull);
    let ch4 = PwmPin::<_, Ch4>::new(m1, OutputType::PushPull);

    let mut pwm = SimplePwm::new(
        tim,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.ch1().max_duty_cycle();
    defmt::info!("motor_task: max_duty={} (expect 667 at 200 MHz / 300 kHz)", max_duty);

    let (b0, b1) = bit_duties(max_duty as u32);

    let disarm_pkt = encode(0, false);
    let disarm_frm = build_frame(disarm_pkt, b0, b1);
    let all_disarm = build_interleaved([disarm_frm; N_CHAN]);

    let dir_frames = [
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_0)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_1)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_2)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_3)), b0, b1),
    ];
    let dir_buf = build_interleaved(dir_frames);

    let spin_pkt = encode(200, false);
    let spin_frm = build_frame(spin_pkt, b0, b1);
    let all_spin = build_interleaved([spin_frm; N_CHAN]);

    loop {
        defmt::info!("cycle: disarming 3 s");
        for _ in 0u16..3000 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &all_disarm).await;
            Timer::after_micros(50).await;
        }

        defmt::info!("cycle: direction");
        for _ in 0..10 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &dir_buf).await;
            Timer::after_millis(1).await;
        }

        defmt::info!("cycle: disarm 200");
        for _ in 0u16..200 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &all_disarm).await;
            Timer::after_micros(50).await;
        }

        defmt::info!("cycle: spinning throttle=200 for 2 s");
        for _ in 0u16..2000 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &all_spin).await;
            Timer::after_micros(50).await;
        }

        defmt::info!("cycle: disarm 500");
        for _ in 0u16..500 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &all_disarm).await;
            Timer::after_micros(50).await;
        }

        Timer::after_millis(500).await;
        defmt::info!("cycle done");
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);
    let mut led_red   = Output::new(p.PE3, Level::Low, Speed::Low);

    let mut uart = UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();
    let _ = uart.write(b"motor_int_test: embassy_config + P10 executor\r\n").await;

    interrupt::FDCAN1_IT0.set_priority(Priority::P10);
    let p10 = EXECUTOR_P10.start(interrupt::FDCAN1_IT0);

    p10.spawn(motor_task(p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1).unwrap());

    let _ = uart.write(b"motor_task spawned at P10\r\n").await;

    loop {
        led_green.toggle();
        led_red.toggle();
        Timer::after_secs(2).await;
        let _ = uart.write(b"heartbeat\r\n").await;
    }
}
