use core::marker::PhantomData;

///! Dshot driver for the stm32f405 using a timer-backed PWM
use dshot_encoder;

use embassy_stm32::interrupt::typelevel::Binding;
#[rustfmt::skip]
use embassy_stm32::{
    Peri,
    gpio::OutputType,
    time::Hertz,
    timer::{
        Ch1,
        Ch2,
        Ch3,
        Ch4, 
        Channel, 
        Dma, 
        GeneralInstance4Channel, 
        TimerPin, 
        UpDma, 
        low_level::CountingMode,
        simple_pwm::{
            PwmPin,
            SimplePwm
        },
    }
};

use common::hw_abstraction::OutputGroup;

const TRANSMIT_SIZE: usize = 24;

fn construct_command(bit: (u16, u16), mut cmd: u16) -> [u16; TRANSMIT_SIZE] {
    let mut dshot_pwm = [0; _];

    for i in (0..16).rev() {
        dshot_pwm[i] = match cmd & 0x1 {
            0 => bit.0,
            _ => bit.1,
        };
        cmd >>= 1;
    }

    dshot_pwm
}

pub struct DshotDriver<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator<Timer = T>,
{
    pwm: SimplePwm<'d, T>,
    wav: WAV,
    bit: (u16, u16),
}

pub struct DshotDriver2<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator2<Timer = T>,
{
    pwm: SimplePwm<'d, T>,
    wav: WAV,
    bit: (u16, u16),
}

impl<'d, T, WAV> DshotDriver<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator<Timer = T>,
{
    pub fn new(
        timer: Peri<'d, T>,
        pin1: Peri<'d, impl TimerPin<T, Ch1>>,
        pin2: Peri<'d, impl TimerPin<T, Ch2>>,
        pin3: Peri<'d, impl TimerPin<T, Ch3>>,
        pin4: Peri<'d, impl TimerPin<T, Ch4>>,
        wav: WAV,
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
            wav,
            bit: (b0, b1),
        }
    }

    async fn transmit(&mut self, commands: [u16; 4]) {
        let commands = commands.map(|cmd| construct_command(self.bit, cmd));
        self.wav.run_waveform(&mut self.pwm, commands).await
    }
}

impl<'d, T, WAV> DshotDriver2<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator2<Timer = T>,
{
    pub fn new(
        timer: Peri<'d, T>,
        pin1: Peri<'d, impl TimerPin<T, Ch1>>,
        pin2: Peri<'d, impl TimerPin<T, Ch2>>,
        wav: WAV,
        khz: u32,
    ) -> Self {
        let mut pwm = SimplePwm::new(
            timer,
            Some(PwmPin::new(pin1, OutputType::PushPull)),
            Some(PwmPin::new(pin2, OutputType::PushPull)),
            None,
            None,
            Hertz::khz(khz),
            CountingMode::EdgeAlignedUp,
        );

        use embassy_stm32::timer::Channel as C;
        for ch in [C::Ch1, C::Ch2] {
            pwm.channel(ch).set_duty_cycle_fully_off();
            pwm.channel(ch).enable();
        }

        let period = pwm.max_duty_cycle() as u32;

        let b0 = ((384 * period) >> 10) as u16;
        let b1 = ((768 * period) >> 10) as u16;

        Self {
            pwm,
            wav,
            bit: (b0, b1),
        }
    }

    async fn transmit(&mut self, commands: [u16; 2]) {
        let commands = commands.map(|cmd| construct_command(self.bit, cmd));
        self.wav.run_waveform(&mut self.pwm, commands).await
    }
}

impl<'d, T, WAV> OutputGroup for DshotDriver<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator<Timer = T>,
{
    async fn set_motor_speeds(&mut self, speed: [u16; 4]) {
        self.transmit(speed.map(|s| dshot_encoder::throttle_clamp(s, false)))
            .await
    }

    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        self.transmit(direction.map(dshot_encoder::reverse)).await
    }

    async fn set_motor_speeds_min(&mut self) {
        self.transmit([dshot_encoder::throttle_minimum(false); 4])
            .await
    }

    async fn make_beep(&mut self) {}
}

impl<'d, T, WAV> OutputGroup for DshotDriver2<'d, T, WAV>
where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator2<Timer = T>,
{
    async fn set_motor_speeds(&mut self, speed: [u16; 4]) {
        self.transmit([
            dshot_encoder::throttle_clamp(speed[0], false),
            dshot_encoder::throttle_clamp(speed[1], false),
        ])
        .await
    }

    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        self.transmit([
            dshot_encoder::reverse(direction[0]),
            dshot_encoder::reverse(direction[1]),
        ])
        .await
    }

    async fn set_motor_speeds_min(&mut self) {
        self.transmit([dshot_encoder::throttle_minimum(false); 2])
            .await
    }

    async fn make_beep(&mut self) {}
}

pub trait WaveformGenerator {
    type Timer: GeneralInstance4Channel;
    async fn run_waveform(
        &mut self,
        pwm: &mut SimplePwm<'_, Self::Timer>,
        cmd: [[u16; TRANSMIT_SIZE]; 4],
    );
}

pub trait WaveformGenerator2 {
    type Timer: GeneralInstance4Channel;
    async fn run_waveform(
        &mut self,
        pwm: &mut SimplePwm<'_, Self::Timer>,
        cmd: [[u16; TRANSMIT_SIZE]; 2],
    );
}

pub struct UpDmaWaveform<'d, T, DMA>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>
{
    dma: Peri<'d, DMA>,
    irq_bind:  BIND,
    _p: PhantomData<T>,
}

impl<'d, T, DMA, BIND> UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>
{
    pub fn new(dma: Peri<'d, DMA>, irq_bind: BIND) -> Self {
        Self {
            dma,
            irq_bind,
            _p: PhantomData,
        }
    }
}

impl<'d, T, DMA, BIND> WaveformGenerator for UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>
{
    type Timer = T;
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: [[u16; TRANSMIT_SIZE]; 4]) {
        let mut interleaved = [0u16; TRANSMIT_SIZE * 4];
        for i in 0..TRANSMIT_SIZE {
            interleaved[i * 4 + 0] = cmd[0][i];
            interleaved[i * 4 + 1] = cmd[1][i];
            interleaved[i * 4 + 2] = cmd[2][i];
            interleaved[i * 4 + 3] = cmd[3][i];
        }

        pwm.waveform_up_multi_channel(
            self.dma.reborrow(),
            self.irq_bind,
            Channel::Ch1,
            Channel::Ch4,
            &interleaved,
        )
        .await;
    }
}

pub struct UpDmaWaveform2<'d, T, DMA>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
{
    dma: Peri<'d, DMA>,
    _p: PhantomData<T>,
}

impl<'d, T, DMA> UpDmaWaveform2<'d, T, DMA>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
{
    pub fn new(dma: Peri<'d, DMA>) -> Self {
        Self {
            dma,
            _p: PhantomData,
        }
    }
}

impl<'d, T, DMA> WaveformGenerator2 for UpDmaWaveform2<'d, T, DMA>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
{
    type Timer = T;
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: [[u16; TRANSMIT_SIZE]; 2]) {
        let mut interleaved = [0u16; TRANSMIT_SIZE * 2];
        for i in 0..TRANSMIT_SIZE {
            interleaved[i * 2 + 0] = cmd[0][i];
            interleaved[i * 2 + 1] = cmd[1][i];
        }

        pwm.waveform_up_multi_channel(
            self.dma.reborrow(),
            Channel::Ch1,
            Channel::Ch2,
            &interleaved,
        )
        .await;
    }
}

pub struct QuadDmaWaveform<'d, T, DMA1, DMA2, DMA3, DMA4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
{
    dma1: Peri<'d, DMA1>,
    dma2: Peri<'d, DMA2>,
    dma3: Peri<'d, DMA3>,
    dma4: Peri<'d, DMA4>,
    irq1: BIND1,
    irq2: BIND2,
    irq3: BIND3,
    irq4: BIND4,
    _p: PhantomData<T>,
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4> QuadDmaWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
{
    #[allow(unused)]
    pub fn new(
        dma1: Peri<'d, DMA1>,
        dma2: Peri<'d, DMA2>,
        dma3: Peri<'d, DMA3>,
        dma4: Peri<'d, DMA4>,
        irq1: BIND1,
        irq2: BIND2,
        irq3: BIND3,
        irq4: BIND4,
    ) -> Self {
        Self {
            dma1,
            dma2,
            dma3,
            dma4,
            irq1,
            irq2,
            irq3,
            irq4,
            _p: PhantomData,
        }
    }
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4> WaveformGenerator
    for QuadDmaWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
where
    T: GeneralInstance4Channel,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
    BIND1: Binding<DMA1::Interrupt, embassy_stm32::dma::InterruptHandler<DMA1>>,
    BIND2: Binding<DMA2::Interrupt, embassy_stm32::dma::InterruptHandler<DMA2>>,
    BIND3: Binding<DMA3::Interrupt, embassy_stm32::dma::InterruptHandler<DMA3>>,
    BIND4: Binding<DMA4::Interrupt, embassy_stm32::dma::InterruptHandler<DMA4>>,
{
    type Timer = T;
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: [[u16; TRANSMIT_SIZE]; 4]) {
        pwm.waveform(self.dma1.reborrow(), self.irq1, Channel::Ch1, cmd[0].as_slice()).await;
        pwm.waveform(self.dma2.reborrow(), self.irq2, Channel::Ch2, cmd[1].as_slice()).await;
        pwm.waveform(self.dma3.reborrow(), self.irq3, Channel::Ch3, cmd[2].as_slice()).await;
        pwm.waveform(self.dma4.reborrow(), self.irq4, Channel::Ch4, cmd[3].as_slice()).await;
    }
}