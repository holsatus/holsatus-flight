//! Continuous DShot disarm task -- silences BLHeli32/BlueJay ESCs.
//!
//! Sensor test binaries (baro_data, compass_data, imu_data, sensors) do not
//! use TIM1 or DMA1_CH1 for motor control, but the ESCs beep loudly when
//! they receive no DShot signal.  Spawning this task at startup keeps all
//! four ESCs disarmed and quiet for the lifetime of the binary.
//!
//! Spawn once from `main`:
//!   spawner.spawn(esc_silence::task(p.TIM1, p.PE9, p.PE11, p.PE13, p.PE14, p.DMA1_CH1)).unwrap();

use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::Peri;
use embassy_time::Timer;

use crate::resources::MotorIrqs;

/// Continuously sends DShot300 throttle=0 (disarm) frames to all four motor
/// channels.  Never returns.
#[embassy_executor::task]
pub async fn task(
    tim:  Peri<'static, peripherals::TIM1>,
    pe9:  Peri<'static, peripherals::PE9>,
    pe11: Peri<'static, peripherals::PE11>,
    pe13: Peri<'static, peripherals::PE13>,
    pe14: Peri<'static, peripherals::PE14>,
    mut dma: Peri<'static, peripherals::DMA1_CH1>,
) -> ! {
    let ch1 = PwmPin::<_, Ch1>::new(pe9,  OutputType::PushPull);
    let ch2 = PwmPin::<_, Ch2>::new(pe11, OutputType::PushPull);
    let ch3 = PwmPin::<_, Ch3>::new(pe13, OutputType::PushPull);
    let ch4 = PwmPin::<_, Ch4>::new(pe14, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        tim,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        Hertz::khz(300),
        CountingMode::EdgeAlignedUp,
    );

    let max = pwm.ch1().max_duty_cycle() as u32;
    let b0  = ((max * 384) >> 10) as u16;

    // DShot disarm: throttle=0, telemetry=0 -> packet=0 -> all 16 data bits = 0
    // -> all slots = b0.  Slots 16..24 stay 0 (reset/silence period).
    let mut frame = [0u16; 24];
    for slot in &mut frame[0..16] {
        *slot = b0;
    }

    loop {
        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            pwm.waveform_up_multi_channel(dma.reborrow(), MotorIrqs, ch, ch, &frame).await;
        }
        Timer::after_micros(500).await;
    }
}
