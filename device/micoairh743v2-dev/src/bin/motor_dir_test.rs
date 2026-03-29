//! Motor direction bisect test -- props OFF, observe one motor at a time.
//!
//! Sequence (repeating):
//!   1. Arm all four ESCs (min throttle, ~2.5 s)
//!   2. Send direction commands per MOTOR_REVERSE_FLAGS (MOTOR_2 reversed)
//!   3. Spin each motor individually for 3 s at DShot 200
//!      Motor is announced on UART before spinning.
//!   4. 1 s pause between motors.
//!
//! Expected spin directions (viewed from ABOVE, prop side up):
//!   M4 (MOTOR_0, back-right):  CW  -- no reversal applied
//!   M3 (MOTOR_1, front-right): CCW -- no reversal applied
//!   M2 (MOTOR_2, back-left):   CCW -- ESC reversed (MOTOR_2 flag set)
//!   M1 (MOTOR_3, front-left):  CW  -- no reversal applied
//!
//! If any motor spins in the WRONG direction the mixer and physical spin do not
//! agree. Fix: either correct MOTOR_REVERSE_FLAGS in config.rs or swap any two
//! motor phase wires on that ESC/motor.
//!
//! If all directions are correct but the drone still cannot lift, the issue is
//! BASE_THRUST -- increase it in alt_hold.rs and re-flash flight.rs.
//!
//! WARNING: remove propellers before running this binary.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, peripherals, Peri};
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

#[path = "../config.rs"]
mod config;
use config::{Reverse, MOTOR_REVERSE_FLAGS};

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => DmaInterruptHandler<peripherals::DMA1_CH0>;
    DMA1_STREAM1 => DmaInterruptHandler<peripherals::DMA1_CH1>;
    USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
});

const SLOTS: usize = 24;
const NCHAN: usize = 4;

fn encode(throttle: u16) -> u16 {
    let value = throttle << 1;
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

fn interleave(frames: [[u16; SLOTS]; NCHAN]) -> [u16; SLOTS * NCHAN] {
    let mut buf = [0u16; SLOTS * NCHAN];
    for t in 0..SLOTS {
        buf[t * NCHAN + 0] = frames[0][t];
        buf[t * NCHAN + 1] = frames[1][t];
        buf[t * NCHAN + 2] = frames[2][t];
        buf[t * NCHAN + 3] = frames[3][t];
    }
    buf
}

async fn burst(
    pwm: &mut SimplePwm<'_, peripherals::TIM1>,
    dma: &mut Peri<'_, peripherals::DMA1_CH1>,
    buf: &[u16; SLOTS * NCHAN],
    count: u32,
    gap_us: u64,
) {
    for _ in 0..count {
        pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, buf).await;
        Timer::after_micros(gap_us).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(config::embassy_config());

    let mut led_green = Output::new(p.PE2, Level::Low, Speed::Low);

    let mut uart = UartTx::new(p.USART1, p.PA9, p.DMA1_CH0, Irqs, UartConfig::default()).unwrap();

    let ch1 = PwmPin::<_, Ch1>::new(p.PE9,  OutputType::PushPull); // M4 silkscreen
    let ch2 = PwmPin::<_, Ch2>::new(p.PE11, OutputType::PushPull); // M3 silkscreen
    let ch3 = PwmPin::<_, Ch3>::new(p.PE13, OutputType::PushPull); // M2 silkscreen
    let ch4 = PwmPin::<_, Ch4>::new(p.PE14, OutputType::PushPull); // M1 silkscreen

    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max = pwm.ch1().max_duty_cycle() as u32;
    let b0 = ((max * 384) >> 10) as u16;
    let b1 = ((max * 768) >> 10) as u16;

    let min_frame  = build_frame(encode(0),   b0, b1);
    let spin_frame = build_frame(encode(200), b0, b1);
    let zero_frame = build_frame(encode(0),   b0, b1);

    let all_min = interleave([min_frame; NCHAN]);

    let rev_frames = [
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_0)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_1)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_2)), b0, b1),
        build_frame(dshot_encoder::reverse(MOTOR_REVERSE_FLAGS.contains(Reverse::MOTOR_3)), b0, b1),
    ];
    let rev_buf = interleave(rev_frames);

    let mut dma = p.DMA1_CH1;

    let _ = uart.write(b"motor_dir_test: remove props, then observe one motor at a time\r\n").await;

    loop {
        // ---- arm all ESCs (~2.5 s) ----
        let _ = uart.write(b"[arm] min throttle 2.5 s ...\r\n").await;
        burst(&mut pwm, &mut dma, &all_min, 2500, 500).await;

        // ---- direction commands ----
        let _ = uart.write(b"[dir] sending direction commands ...\r\n").await;
        for _ in 0..20 {
            pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &rev_buf).await;
            Timer::after_millis(1).await;
        }

        // brief disarm gap after direction command
        burst(&mut pwm, &mut dma, &all_min, 200, 500).await;

        // ---- spin each motor one at a time ----
        const MOTOR_INFO: [(&[u8], &[u8]); 4] = [
            (b"[M4] MOTOR_0 back-right  -- expected CW\r\n",               b"[M4] done\r\n"),
            (b"[M3] MOTOR_1 front-right -- expected CCW\r\n",              b"[M3] done\r\n"),
            (b"[M2] MOTOR_2 back-left   -- expected CCW (ESC reversed)\r\n", b"[M2] done\r\n"),
            (b"[M1] MOTOR_3 front-left  -- expected CW\r\n",               b"[M1] done\r\n"),
        ];

        for (ch_idx, (start_msg, end_msg)) in MOTOR_INFO.iter().enumerate() {
            // 1 s min-throttle gap between motors
            burst(&mut pwm, &mut dma, &all_min, 200, 500).await;

            let _ = uart.write(start_msg).await;
            led_green.set_high();

            // only the active channel spins; rest get zero throttle
            let mut solo = [zero_frame; NCHAN];
            solo[ch_idx] = spin_frame;
            let solo_buf = interleave(solo);

            burst(&mut pwm, &mut dma, &solo_buf, 3000, 500).await;

            led_green.set_low();
            let _ = uart.write(end_msg).await;
        }

        let _ = uart.write(b"[cycle] complete -- restarting\r\n\r\n").await;
        Timer::after_secs(2).await;
    }
}
