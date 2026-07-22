use core::marker::PhantomData;

///! Dshot driver for the stm32f405 using a timer-backed PWM
use dshot_encoder;

use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{
    timer::{
        simple_pwm::SimplePwm, Ch1, Ch2, Ch3, Ch4, Dma, GeneralInstance4Channel, TimerPin, UpDma,
    },
    Peri,
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
        use embassy_stm32::gpio::OutputType;
        use embassy_stm32::time::Hertz;
        use embassy_stm32::timer::{low_level::CountingMode, simple_pwm::PwmPin};
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
        self.wav.run_waveform(&mut self.pwm, &commands).await
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

pub trait WaveformGenerator {
    type Timer: GeneralInstance4Channel;
    fn run_waveform(
        &mut self,
        pwm: &mut SimplePwm<'_, Self::Timer>,
        cmd: &[[u16; TRANSMIT_SIZE]; 4],
    ) -> impl core::future::Future<Output = ()>;
}

pub struct UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    dma: Peri<'d, DMA>,
    irq: BIND,
    _p: PhantomData<T>,
}

impl<'d, T, DMA, BIND> UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    pub fn new(dma: Peri<'d, DMA>, irq: BIND) -> Self {
        Self {
            dma,
            irq,
            _p: PhantomData,
        }
    }
}

impl<'d, T, DMA, BIND> WaveformGenerator for UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    type Timer = T;
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: &[[u16; TRANSMIT_SIZE]; 4]) {
        let mut interleaved = [0u16; TRANSMIT_SIZE * 4];
        for i in 0..TRANSMIT_SIZE {
            interleaved[i * 4 + 0] = cmd[0][i];
            interleaved[i * 4 + 1] = cmd[1][i];
            interleaved[i * 4 + 2] = cmd[2][i];
            interleaved[i * 4 + 3] = cmd[3][i];
        }

        use embassy_stm32::timer::Channel;
        pwm.waveform_up_multi_channel(
            self.dma.reborrow(),
            self.irq,
            Channel::Ch1,
            Channel::Ch4,
            &interleaved,
        )
        .await;
    }
}

#[allow(unused)]
pub struct QuadDmaQuadIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
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

impl<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
    QuadDmaQuadIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
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
    for QuadDmaQuadIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND1, BIND2, BIND3, BIND4>
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
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: &[[u16; TRANSMIT_SIZE]; 4]) {
        use embassy_stm32::timer::Channel;
        pwm.waveform(
            self.dma1.reborrow(),
            self.irq1,
            Channel::Ch1,
            cmd[0].as_slice(),
        )
        .await;
        pwm.waveform(
            self.dma2.reborrow(),
            self.irq2,
            Channel::Ch2,
            cmd[1].as_slice(),
        )
        .await;
        pwm.waveform(
            self.dma3.reborrow(),
            self.irq3,
            Channel::Ch3,
            cmd[2].as_slice(),
        )
        .await;
        pwm.waveform(
            self.dma4.reborrow(),
            self.irq4,
            Channel::Ch4,
            cmd[3].as_slice(),
        )
        .await;
    }
}

#[allow(unused)]
pub struct QuadDmaSingleIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND>
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
    irq: BIND,
    _p: PhantomData<T>,
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4, BIND>
    QuadDmaSingleIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND>
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
        irq: BIND,
    ) -> Self {
        Self {
            dma1,
            dma2,
            dma3,
            dma4,
            irq,
            _p: PhantomData,
        }
    }
}

impl<'d, T, DMA1, DMA2, DMA3, DMA4, BIND> WaveformGenerator
    for QuadDmaSingleIrqWaveform<'d, T, DMA1, DMA2, DMA3, DMA4, BIND>
where
    T: GeneralInstance4Channel + embassy_stm32::timer::CoreInstance,
    DMA1: Dma<T, Ch1>,
    DMA2: Dma<T, Ch2>,
    DMA3: Dma<T, Ch3>,
    DMA4: Dma<T, Ch4>,
    BIND: Binding<DMA1::Interrupt, embassy_stm32::dma::InterruptHandler<DMA1>>
        + Binding<DMA2::Interrupt, embassy_stm32::dma::InterruptHandler<DMA2>>
        + Binding<DMA3::Interrupt, embassy_stm32::dma::InterruptHandler<DMA3>>
        + Binding<DMA4::Interrupt, embassy_stm32::dma::InterruptHandler<DMA4>>,
{
    type Timer = T;
    async fn run_waveform(&mut self, _pwm: &mut SimplePwm<'_, T>, cmd: &[[u16; TRANSMIT_SIZE]; 4]) {
        let base_ptr = T::regs() as *mut u8;

        // This is a bit too brute-force for my liking, investigate usage of safer APIs
        // It seems to work, but do see reference manual for address offsets, starting at page 1587
        // https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

        let req1 = self.dma1.request();
        let req2 = self.dma2.request();
        let req3 = self.dma3.request();
        let req4 = self.dma4.request();

        // Create the 4 DMA channels explicitly
        let mut ch1 = embassy_stm32::dma::Channel::new(self.dma1.reborrow(), self.irq);
        let mut ch2 = embassy_stm32::dma::Channel::new(self.dma2.reborrow(), self.irq);
        let mut ch3 = embassy_stm32::dma::Channel::new(self.dma3.reborrow(), self.irq);
        let mut ch4 = embassy_stm32::dma::Channel::new(self.dma4.reborrow(), self.irq);

        let (tx1, tx2, tx3, tx4) = unsafe {
            // Calculate raw pointers to control registers
            let ccr1_ptr = base_ptr.add(0x34) as *mut u16;
            let ccr2_ptr = base_ptr.add(0x38) as *mut u16;
            let ccr3_ptr = base_ptr.add(0x3C) as *mut u16;
            let ccr4_ptr = base_ptr.add(0x40) as *mut u16;

            let dma_opts = embassy_stm32::dma::TransferOptions::default();

            // Construct the asynchronous DMA transfer futures
            let tx1 = ch1.write(req1, &cmd[0], ccr1_ptr, dma_opts);
            let tx2 = ch2.write(req2, &cmd[1], ccr2_ptr, dma_opts);
            let tx3 = ch3.write(req3, &cmd[2], ccr3_ptr, dma_opts);
            let tx4 = ch4.write(req4, &cmd[3], ccr4_ptr, dma_opts);

            (tx1, tx2, tx3, tx4)
        };

        // Configure hardware to trigger DMA on the update event
        unsafe {
            let cr2_ptr = base_ptr.add(0x04) as *mut u32;
            let cr2_val = core::ptr::read_volatile(cr2_ptr);
            core::ptr::write_volatile(cr2_ptr, cr2_val | (1 << 3));
        }

        // Enable capture/compare (CCxDE) in DMA/interrupt enable register
        unsafe {
            let dier_ptr = base_ptr.add(0x0C) as *mut u32;
            let dier_val = core::ptr::read_volatile(dier_ptr);
            core::ptr::write_volatile(dier_ptr, dier_val | (0b1111 << 9));
        }

        // Await all DMA streams simultaneously
        embassy_futures::join::join4(tx1, tx2, tx3, tx4).await;

        // Disable capture/compare (CCxDE) in DMA/interrupt enable register
        unsafe {
            let dier_ptr = base_ptr.add(0x0C) as *mut u32;
            let dier_val = core::ptr::read_volatile(dier_ptr);
            core::ptr::write_volatile(dier_ptr, dier_val & !(0b1111 << 9));
        }
    }
}
