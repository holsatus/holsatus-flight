///! Dshot driver for the stm32f405 using a timer-backed PWM

use dshot_encoder;
use embassy_stm32::{
    gpio::OutputType,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
        Ch1Dma, Ch2Dma, Ch3Dma, Ch4Dma, Channel1Pin, Channel2Pin, Channel3Pin, Channel4Pin,
        GeneralInstance4Channel,
    },
    Peripheral,
};

use common::hw_abstraction::FourMotors;

pub struct DshotPwm<'d, T, D1, D2, D3, D4>
where
    T: GeneralInstance4Channel,
{
    pwm: SimplePwm<'d, T>,
    dma1: D1,
    dma2: D2,
    dma3: D3,
    dma4: D4,
    bit: (u16, u16),
}

impl<'d, T, D1, DA1, D2, DA2, D3, DA3, D4, DA4> DshotPwm<'d, T, D1, D2, D3, D4>
where
    T: GeneralInstance4Channel,
    D1: Peripheral<P = DA1> + 'd,
    DA1: Ch1Dma<T>,
    D2: Peripheral<P = DA2> + 'd,
    DA2: Ch2Dma<T>,
    D3: Peripheral<P = DA3> + 'd,
    DA3: Ch3Dma<T>,
    D4: Peripheral<P = DA4> + 'd,
    DA4: Ch4Dma<T>,
{
    pub fn new(
        timer: impl Peripheral<P = T> + 'd,
        pin1: &'d mut impl Channel1Pin<T>,
        pin2: &'d mut impl Channel2Pin<T>,
        pin3: &'d mut impl Channel3Pin<T>,
        pin4: &'d mut impl Channel4Pin<T>,
        dma1: D1,
        dma2: D2,
        dma3: D3,
        dma4: D4,
        khz: u32,
    ) -> Self {
        let mut pwm = SimplePwm::new(
            timer,
            Some(PwmPin::new_ch1(pin1, OutputType::PushPull)),
            Some(PwmPin::new_ch2(pin2, OutputType::PushPull)),
            Some(PwmPin::new_ch3(pin3, OutputType::PushPull)),
            Some(PwmPin::new_ch4(pin4, OutputType::PushPull)),
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

        self.pwm.waveform_ch1(&mut self.dma1, command0.as_slice()).await;
        self.pwm.waveform_ch2(&mut self.dma2, command1.as_slice()).await;
        self.pwm.waveform_ch3(&mut self.dma3, command2.as_slice()).await;
        self.pwm.waveform_ch4(&mut self.dma4, command3.as_slice()).await;
    }

    fn construct_command(&self, cmd: u16) -> [u16; 17] {
        // The extra bit ensures the channel is kept low (0 duty cycle) after transmission
        let mut dshot_pwm = [0; 17];

        dshot_pwm
            .iter_mut()
            .take(16) // Leave 1 bit low at the end
            .rev()
            .enumerate()
            .for_each(|(i, v)| 
                match (cmd >> i) & 0x1 {
                    0 => *v = self.bit.0,
                    _ => *v = self.bit.1,
                }
            );
        dshot_pwm
    }
}

impl<'d, T, D1, DA1, D2, DA2, D3, DA3, D4, DA4> FourMotors for DshotPwm<'d, T, D1, D2, D3, D4>
where
    T: GeneralInstance4Channel,
    D1: Peripheral<P = DA1> + 'd,
    DA1: Ch1Dma<T>,
    D2: Peripheral<P = DA2> + 'd,
    DA2: Ch2Dma<T>,
    D3: Peripheral<P = DA3> + 'd,
    DA3: Ch3Dma<T>,
    D4: Peripheral<P = DA4> + 'd,
    DA4: Ch4Dma<T>,
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

    async fn make_beep(&mut self) {
        todo!()
    }
}
