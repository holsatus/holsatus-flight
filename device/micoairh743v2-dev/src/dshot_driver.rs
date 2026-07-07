//! DShot driver for the STM32H743 using TIM1 with UP-DMA waveform output.
//!
//! Motor assignments (same as motor_dshot.rs test):
//!   M1 = PE14 (TIM1 CH4)
//!   M2 = PE13 (TIM1 CH3)
//!   M3 = PE11 (TIM1 CH2)
//!   M4 = PE9  (TIM1 CH1)
//!
//! DMA: DMA1_CH1 drives TIM1 UP (confirmed working in motor_dshot.rs).
//!
//! This module is a direct adaptation of stm32f405-dev/src/dshot_pwm.rs.
//! TIM1 on H743 implements GeneralInstance4Channel (advanced timers are a superset),
//! so the same generic driver compiles against it without modification.

use core::marker::PhantomData;

use dshot_encoder;

use embassy_stm32::interrupt::typelevel::Binding;
#[rustfmt::skip]
use embassy_stm32::{
    Peri,
    gpio::OutputType,
    time::Hertz,
    timer::{
        Ch1, Ch2, Ch3, Ch4,
        Channel,
        GeneralInstance4Channel,
        TimerPin,
        UpDma,
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
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

        // T0H = 37.5% of bit period: (2.5/6.66)*1024 = 384
        let b0 = ((384 * period) >> 10) as u16;

        // T1H = 75% of bit period: (5.0/6.66)*1024 = 768
        let b1 = ((768 * period) >> 10) as u16;

        Self { pwm, wav, bit: (b0, b1) }
    }

    async fn transmit(&mut self, commands: [u16; 4]) {
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
        self.transmit(speed.map(|s| dshot_encoder::throttle_clamp(s, false))).await
    }

    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        self.transmit(direction.map(dshot_encoder::reverse)).await
    }

    async fn set_motor_speeds_min(&mut self) {
        self.transmit([0u16; 4]).await
    }

    async fn make_beep(&mut self) {
        // DShot beacon: the ESC plays a tone WITHOUT spinning the motor, so
        // this is safe at any orientation and needs no arming. Beep5 is the
        // longest/loudest tone (~1020 ms). The caller must pace bursts at least
        // that far apart (see dshot_keepalive_sender).
        let frame = dshot_encoder::command(dshot_encoder::DshotCmdT::DigitalCmdBeep5);
        self.transmit([frame; 4]).await
    }
}

/// Find-my-drone request: set by `rc_loss_beacon_task` when RC has been gone
/// for >30 s. While set, the motor-output loops pulse the DShot beacon (a tone,
/// no motor spin). Honoured in BOTH `pre_arm_loop` (so a crash-rebooted,
/// never-armed drone still beeps) and `dshot_keepalive_sender` (lost after
/// flying). The keepalive side additionally gates on motors-idle.
pub static RC_BEACON_ACTIVE: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Hard motor-kill latch, enforced at the DShot keepalive -- the lowest
/// software layer that touches the motors. When set, the keepalive streams
/// DShot 0 (true disarm) regardless of what DSHOT_SPEEDS holds, so motors
/// stop even if the motor governor task is wedged and deaf to
/// COMMAD_ARM_VEHICLE (2026-07-08 incident: FSM + rc_kill + flip_kill all
/// sent disarms on the watch, the governor consumed none of them, and the
/// keepalive kept streaming the last speeds; the only remaining stop was
/// pulling the battery).
///
/// Setters: rc_kill (SE kill, SF restart), flip_kill, gyro_runaway_kill.
/// Set-once; cleared only by reboot (SF restart path). Setters MUST store
/// this latch before any await, log, or watch send -- a plain atomic store
/// is the one operation that cannot be blocked by a wedged task, a full
/// log channel, or a lost wake.
pub static MOTOR_KILL: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Loop DShot keep-alive batches until the arm predicate returns `true`.
///
/// Each iteration sends ~3000 DShot-0 frames (~390 ms at ~7.7 kHz) and then
/// checks `is_armed`. The check happens after the burst, so the ESC always
/// receives at least one batch regardless of when the LiPo is connected.
///
/// BLHeli/BlueJay require a gap-free DShot-0 stream to recognise the FC.
/// This loop replaces the old single-shot approach (narrow ~400 ms boot window)
/// with a continuous stream that works at any LiPo connection time.
///
/// While RC_BEACON_ACTIVE (RC gone >30 s, always disarmed here), it pulses the
/// DShot beacon -- a ~10 ms burst every ~1.5 s -- so a crash-rebooted, never-
/// armed drone is still findable by ear. The beacon never spins the motor.
pub async fn pre_arm_loop<T, WAV>(
    driver: &mut DshotDriver<'_, T, WAV>,
    mut is_armed: impl FnMut() -> bool,
) where
    T: GeneralInstance4Channel,
    WAV: WaveformGenerator<Timer = T>,
{
    // Cadence in frames. Measured ~400 us/frame here (the DMA waveform
    // dominates, not the 50 us timer): 30000 frames was ~12 s, so 7500 ~= 3 s.
    // Burst 500 frames (~200 ms) sends the beacon long enough to reliably
    // trigger the tone every cycle.
    const BEACON_PERIOD: u32 = 7_500;
    const BEACON_BURST: u32 = 500;
    let mut frame: u32 = 0;
    loop {
        for _ in 0u32..3_000 {
            if RC_BEACON_ACTIVE.load(core::sync::atomic::Ordering::Relaxed)
                && frame % BEACON_PERIOD < BEACON_BURST
            {
                driver.make_beep().await;
            } else {
                driver.set_motor_speeds_min().await;
            }
            frame = frame.wrapping_add(1);
            embassy_time::Timer::after_micros(50).await;
        }
        if is_armed() {
            return;
        }
    }
}

/// Global DShot frame counter -- incremented each time run_waveform completes.
/// Read from motor_monitor to confirm frames are actually being transmitted.
pub static DSHOT_TX_COUNT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub trait WaveformGenerator {
    type Timer: GeneralInstance4Channel;
    async fn run_waveform(
        &mut self,
        pwm: &mut SimplePwm<'_, Self::Timer>,
        cmd: [[u16; TRANSMIT_SIZE]; 4],
    );
}

pub struct UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    dma: Peri<'d, DMA>,
    irq_bind: BIND,
    _p: PhantomData<T>,
}

impl<'d, T, DMA, BIND> UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    pub fn new(dma: Peri<'d, DMA>, irq_bind: BIND) -> Self {
        Self { dma, irq_bind, _p: PhantomData }
    }
}

impl<'d, T, DMA, BIND> WaveformGenerator for UpDmaWaveform<'d, T, DMA, BIND>
where
    T: GeneralInstance4Channel,
    DMA: UpDma<T>,
    BIND: Binding<DMA::Interrupt, embassy_stm32::dma::InterruptHandler<DMA>>,
{
    type Timer = T;
    async fn run_waveform(&mut self, pwm: &mut SimplePwm<'_, T>, cmd: [[u16; TRANSMIT_SIZE]; 4]) {
        // Send each channel as a separate single-channel DMA burst (DBL=0).
        //
        // Using waveform_up_multi_channel(Ch1, Ch4, interleaved) appears correct
        // but the embassy DMA options use pburst=Single, meaning only 1 CCR is
        // updated per timer UEV.  The burst counter cycles CCR1->CCR2->CCR3->CCR4
        // making each channel's effective DShot rate 300/4 = 75 kHz.  Sending
        // four sequential single-channel bursts gives correct 300 kHz per channel.
        let channels = [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4];
        for (ch, frame) in channels.iter().zip(cmd.iter()) {
            pwm.waveform_up_multi_channel(
                self.dma.reborrow(),
                self.irq_bind,
                *ch,
                *ch,
                frame,
            )
            .await;
        }
        DSHOT_TX_COUNT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
}
