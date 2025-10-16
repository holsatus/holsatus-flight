///! Dshot driver for the stm32f405 using a timer-backed PWM
use dshot_encoder;
use embassy_stm32::{
    gpio::OutputType,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
        Ch1, Ch2, Ch3, Ch4, Dma, GeneralInstance4Channel, TimerPin,
    },
    Peri,
};

use common::hw_abstraction::OutputGroup;

pub struct DshotPwm<'d, T, DMA1, DMA2, DMA3, DMA4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
{
    pwm: SimplePwm<'d, T>,
    dma1: Peri<'d, DMA1>,
    dma2: Peri<'d, DMA2>,
    dma3: Peri<'d, DMA3>,
    dma4: Peri<'d, DMA4>,
    bit: (u16, u16),
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4> DshotPwm<'d, T, DMA1, DMA2, DMA3, DMA4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
{
    pub fn new(
        timer: Peri<'d, T>,
        pin1: Peri<'d, impl TimerPin<T, Ch1>>,
        pin2: Peri<'d, impl TimerPin<T, Ch2>>,
        pin3: Peri<'d, impl TimerPin<T, Ch3>>,
        pin4: Peri<'d, impl TimerPin<T, Ch4>>,
        dma1: Peri<'d, DMA1>,
        dma2: Peri<'d, DMA2>,
        dma3: Peri<'d, DMA3>,
        dma4: Peri<'d, DMA4>,
        khz: u32,
    ) -> Self {
        let mut pwm = SimplePwm::new(
            timer,
            Some(PwmPin::new(pin1, OutputType::PushPull)),
            Some(PwmPin::new(pin2, OutputType::PushPull)),
            Some(PwmPin::new(pin3, OutputType::PushPull)),
            Some(PwmPin::new(pin4, OutputType::PushPull)),
            Hertz::khz(khz),
            CountingMode::EdgeAlignedUp,
        );

        use embassy_stm32::timer::Channel as C;
        for ch in [C::Ch1, C::Ch2, C::Ch3, C::Ch4] {
            pwm.channel(ch).set_duty_cycle_fully_off();
            pwm.channel(ch).enable();
        }

        let period = pwm.max_duty_cycle() as u32;

        // Magic number is calculated as (T0H/BIT)*2^10 => (2.5/6.66)*1024 = 384
        let b0 = ((384 * period) >> 10) as u16;

        // Magic number is calculated as (T1H/BIT)*2^10 => (5.0/6.66)*1024 = 768
        let b1 = ((768 * period) >> 10) as u16;

        Self {
            pwm,
            dma1,
            dma2,
            dma3,
            dma4,
            bit: (b0, b1),
        }
    }

    async fn transmit(&mut self, commands: (u16, u16, u16, u16)) {
        let command0 = self.construct_command(commands.0);
        let command1 = self.construct_command(commands.1);
        let command2 = self.construct_command(commands.2);
        let command3 = self.construct_command(commands.3);

        // Cannot use join(f,f,f,f) here since the pwm/timer type is borrowed mutably
        self.pwm
            .waveform(self.dma1.reborrow(), command0.as_slice())
            .await;
        self.pwm
            .waveform(self.dma2.reborrow(), command1.as_slice())
            .await;
        self.pwm
            .waveform(self.dma3.reborrow(), command2.as_slice())
            .await;
        self.pwm
            .waveform(self.dma4.reborrow(), command3.as_slice())
            .await;
    }

    fn construct_command(&self, cmd: u16) -> [u16; 17] {
        // The extra bit ensures the channel is kept low (0 duty cycle) after transmission
        let mut dshot_pwm = [0; 17];

        dshot_pwm
            .iter_mut()
            .take(16) // Leave 1 bit low at the end
            .rev()
            .enumerate()
            .for_each(|(i, v)| match (cmd >> i) & 0x1 {
                0 => *v = self.bit.0,
                _ => *v = self.bit.1,
            });
        dshot_pwm
    }
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4> OutputGroup for DshotPwm<'d, T, DMA1, DMA2, DMA3, DMA4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
{
    async fn set_motor_speeds(&mut self, speed: [u16; 4]) {
        self.transmit((
            dshot_encoder::throttle_clamp(speed[0], false),
            dshot_encoder::throttle_clamp(speed[1], false),
            dshot_encoder::throttle_clamp(speed[2], false),
            dshot_encoder::throttle_clamp(speed[3], false),
        ))
        .await
    }

    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        self.transmit((
            dshot_encoder::reverse(direction[0]),
            dshot_encoder::reverse(direction[1]),
            dshot_encoder::reverse(direction[2]),
            dshot_encoder::reverse(direction[3]),
        ))
        .await
    }

    async fn set_motor_speeds_min(&mut self) {
        self.transmit((
            dshot_encoder::throttle_minimum(false),
            dshot_encoder::throttle_minimum(false),
            dshot_encoder::throttle_minimum(false),
            dshot_encoder::throttle_minimum(false),
        ))
        .await
    }

    async fn make_beep(&mut self) {}
}
