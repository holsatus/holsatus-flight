//! Peripheral resource splitting and embassy task wrappers for the MicoAir H743.
//!
//! Peripheral assignments:
//!   BMI088 IMU  -- SPI2:  SCLK=PD3, MOSI=PC3, MISO=PC2, CS_ACC=PD4, CS_GYR=PD5
//!                          DMA1_CH6 (TX), DMA1_CH7 (RX)
//!   DPS310 Baro -- I2C2:  SCL=PB10, SDA=PB11, addr=0x76 (SDO low)
//!   IST8310 Mag -- I2C2:  shared bus, addr=0x0E (fixed)
//!                          DMA1_CH4 (TX), DMA1_CH5 (RX)
//!   Motors      -- TIM1:  CH1=PE9(M4), CH2=PE11(M3), CH3=PE13(M2), CH4=PE14(M1)
//!                          DMA1_CH1 (UP DMA)
//!   USART1      -- TX=PA9, DMA1_CH0
//!   GPS (USART3) -- RX=PD9, TX=PD8, DMA1_CH2

use assign_resources::assign_resources;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::Config as I2cConfig;
use embassy_stm32::spi;
use embassy_stm32::spi::{mode::Master as SpiMaster, Config as SpiConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{mode::Async, peripherals, Peri, Peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use common::hw_abstraction::OutputGroup;
use common::types::config::DshotConfig;

use crate::bmi088::Bmi088;
use crate::bmi088_imu6dof::Bmi088Imu6Dof;
use crate::dshot_driver::{DshotDriver, UpDmaWaveform};

// ================================================================================
// QMC5883L magnetometer calibration (single source of truth).
//
// Every binary that uses the compass (flight, sub_hover_test, ...)
// goes through `compass_reader` below, which applies these constants. To
// re-calibrate after a hardware change (battery/cable re-route, new FC mount):
//   1. Flash `mag_cal` binary, capture 120 s of rotation per run
//   2. Combine CSVs via `python3 tools/mag_cal_fit.py D*/000001.CSV`
//   3. Paste the ellipsoid output into these two constants
//
// Fit provenance: D000098 (mag_cal_dynamic, motors spinning at DShot=400,
// 8929 samples, 2026-04-23), ellipsoid residual 7.98% of radius. Sphere
// coverage 99.0% within 10 deg, max gap 12.3 deg. +z cardinal 6.2 deg
// (marginal, widen on next run). Higher residual than bench cal D000073
// (4.53%) because motor PWM/commutation adds AC noise; the benefit is
// hard-iron bias now reflects the in-flight magnetic environment.
// Re-fit required whenever battery mount or high-current cable routing
// changes relative to the FC.
//
// Prior bench cal (D000073, motors silent, 4.53% residual) kept here as
// fallback reference:
//   pub const MAG_BIAS_UT: [f32; 3] = [-8.7396e+02, -6.5483e+02, 3.3176e+02];
//   pub const MAG_CAL_MAT: [[f32; 3]; 3] = [
//       [ 6.606139e-04, -1.258174e-05, -5.539052e-06],
//       [-1.258174e-05,  6.938219e-04,  3.041682e-05],
//       [-5.539052e-06,  3.041682e-05,  7.432046e-04],
//   ];
// ================================================================================
pub const MAG_BIAS_UT: [f32; 3] = [-5.1719e+02, 1.4860e+02, 6.2322e+02];
pub const MAG_CAL_MAT: [[f32; 3]; 3] = [
    [7.424621e-04, 5.461799e-07, 2.812429e-05],
    [5.461799e-07, 7.617944e-04, 6.740758e-06],
    [2.812429e-05, 6.740758e-06, 7.475147e-04],
];

assign_resources! {
    imu: ImuResources {
        spi:    SPI2,
        sclk:   PD3,
        mosi:   PC3,
        miso:   PC2,
        cs_acc: PD4,
        cs_gyr: PD5,
        dma_tx: DMA1_CH6,
        dma_rx: DMA1_CH7,
    }
    baro: BaroResources {
        i2c:    I2C2,
        scl:    PB10,
        sda:    PB11,
        dma_tx: DMA1_CH4,
        dma_rx: DMA1_CH5,
    }
    motors: MotorResources {
        tim: TIM1,
        m1:  PE14,
        m2:  PE13,
        m3:  PE11,
        m4:  PE9,
        dma: DMA1_CH1,
    }
    // LED pin-to-color mapping validated on the REPLACEMENT board
    // (2026-07, after the original FC died). The old board had
    // red=PE3 / blue=PE4 / green=PE2; on the replacement the three
    // LEDs are rotated. Evidence: D000001 bench test -- the Ready
    // rendering (drives `blue`) appeared green and the Blocked
    // rendering (drives `red`) appeared blue. If a future board
    // swap changes colors again, re-verify with led.rs modes.
    leds: LedResources {
        red:   PE2,
        blue:  PE3,
        green: PE4,
    }
    uart_log: UartLogResources {
        usart: USART1,
        tx:    PA9,
        rx: PA10,
        dma:   DMA1_CH0,
        dma_rx: DMA2_CH7,
    }
    /// Onboard Bluetooth telemetry module. UART8 TX=PE1, RX=PE0 per the
    /// MicoAir H743v2 spec sheet. We only use TX -- the BT module bridges
    /// UART <-> BT-Classic SPP and exposes itself to paired hosts as
    /// "MicoAir743v2-XXXXX" at 115200 baud. Mirrors USART1's debug log.
    bt_log: BtLogResources {
        usart: UART8,
        tx:    PE1,
        dma:   DMA2_CH3,
    }
    battery: BatteryResources {
        adc:   ADC1,
        pin_v: PC0,
    }
    mtf01: Mtf01Resources {
        usart: UART4,
        rx:    PA1,
        dma:   DMA1_CH3,
    }
    gps: GpsResources {
        usart: USART3,
        rx:    PD9,
        tx:    PD8,
        dma:   DMA1_CH2,
    }
    rc: RcResources {
        usart: USART6,
        rx:    PC7,
        dma:   DMA2_CH2,
    }
    /// Open Drone ID transmitter. UART5 TX=PB6 at 57600 baud feeds a Waveshare
    /// ESP32-S3-Zero "Supermini" flashed with ArduRemoteID v1.14. The bridge
    /// is TX-only; the module does not talk back over MAVLink. See
    /// `tools/odid_emit/odid_emit.py` for the Python reference flow.
    odid: OdidResources {
        usart: UART5,
        tx:    PB6,
        dma:   DMA2_CH4,
    }
    sdmmc: SdmmcLogResources {
        periph: SDMMC1,
        clk:    PC12,
        cmd:    PD2,
        d0:     PC8,
        d1:     PC9,
        d2:     PC10,
        d3:     PC11,
    }
    bmi270: Bmi270Resources {
        spi:    SPI3,
        sclk:   PB3,
        mosi:   PD6,
        miso:   PB4,
        cs:     PA15,
        dma_tx: DMA2_CH1,
        dma_rx: DMA2_CH0,
    }
    /// Phoenix Pi-companion MAVLink telemetry link over USART2 on the board's
    /// DJI-VTX 6-pin connector. Pinout on that header: 12V/GND/Tx2/Rx2/GND/Rx6;
    /// only Tx2 (PA2), Rx2 (PA3), and one GND are wired through to the RPi5.
    /// Repurposes the FPV HD-VTX port -- analog VTX and DJI O3 are mutually
    /// exclusive with the Pi-side mission logger on this drone. See
    /// `phoenix_telem` and references/holsatus_data_logging.md for the wire
    /// protocol.
    telem: TelemResources {
        usart:  USART2,
        tx:     PA2,
        rx:     PA3,
        dma_tx: DMA2_CH5,
        dma_rx: DMA2_CH6,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}

// Module-level interrupt bindings, one struct per peripheral instance (bus).
// Defined here rather than inside individual tasks so that every binary
// touching a given peripheral -- whether via a lib task or directly --
// shares the same bind_interrupts! struct.
//
// `bind_interrupts!` expands to `#[unsafe(no_mangle)] extern "C" fn <VECTOR>()`,
// a flat linker symbol with no per-crate/per-module namespacing (required so
// the hardware vector table can find it by exact name). Two DIFFERENT
// bind_interrupts! structs binding the SAME vector name is therefore a hard
// error the moment both are compiled into the same crate -- and since this
// whole file is unconditionally part of the shared `micoairh743v2` lib (which
// every single binary in this crate links against), that check happens for
// EVERY vector bound here, for EVERY binary, regardless of which one actually
// uses it. So: any vector genuinely shared across binaries belongs here,
// bound exactly once.
//
// This constraint turned out to be stronger than "shared vectors go here,
// one-off vectors stay local": once a binary references ANYTHING in this
// lib crate (which is effectively every binary, via `config::embassy_config()`),
// EVERY `#[no_mangle]` interrupt handler declared anywhere in the lib becomes
// part of that binary's final link -- whether or not that binary's own code
// path ever reaches it. A binary can therefore never declare its OWN local
// binding for a vector already bound here, even for a peripheral that has
// nothing to do with the vector's "owning" struct. That's why SensorIrqs
// below bundles GPS + MTF-01 + esc_telemetry's UART7 even though they're
// functionally unrelated: DMA1_STREAM2 and DMA1_STREAM3 are each claimed
// exactly once, crate-wide, and every binary touching any of these
// peripherals has to share the one struct that owns them.

// USART1: debug log, TX on nearly every binary (DMA1_CH0), full-duplex RX
// (DMA2_CH7) only for hil_echo's loopback. One struct covers both so any
// binary can request either mode with the same value.
bind_interrupts!(pub struct UartLogIrqs {
    USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
    DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH7>;
});

// GPS (USART3, MicoAir MG-A01 ublox module, DMA1_CH2) + MTF-01 optical flow
// (UART4, DMA1_CH3) + esc_telemetry.rs's bench listener (UART7, reusing
// GPS's DMA1_CH2 since it's never run alongside GPS) + flow_data.rs's bench
// wiring (MTF-01 physically wired to the USART3 header, read via DMA1_CH3).
// See the comment above this section for why these unrelated peripherals
// share one struct instead of each getting their own.
bind_interrupts!(pub struct SensorIrqs {
    USART3       => embassy_stm32::usart::InterruptHandler<peripherals::USART3>;
    DMA1_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
    UART4        => embassy_stm32::usart::InterruptHandler<peripherals::UART4>;
    DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
    UART7        => embassy_stm32::usart::InterruptHandler<peripherals::UART7>;
});

// Onboard Bluetooth telemetry bridge (UART8, TX only) -- DMA2_CH3.
bind_interrupts!(pub struct BtLogIrqs {
    UART8        => embassy_stm32::usart::InterruptHandler<peripherals::UART8>;
    DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
});

bind_interrupts!(pub struct I2c2Irqs {
    I2C2_EV      => embassy_stm32::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER      => embassy_stm32::i2c::ErrorInterruptHandler<peripherals::I2C2>;
    DMA1_STREAM4 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_STREAM5 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH5>;
});

bind_interrupts!(pub struct Spi2Irqs {
    DMA1_STREAM6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
});

bind_interrupts!(pub struct MotorIrqs {
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
});

// SPI3 (BMI270): DMA2_CH0 (RX), DMA2_CH1 (TX).
bind_interrupts!(pub struct Spi3Irqs {
    DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH0>;
    DMA2_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH1>;
});

// Open Drone ID transmitter (UART5, TX only) -- DMA2_CH4. See odid.rs.
bind_interrupts!(pub struct OdidIrqs {
    UART5        => embassy_stm32::usart::InterruptHandler<peripherals::UART5>;
    DMA2_STREAM4 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH4>;
});

// SDMMC1 (SD card logging). No DMA vector -- H743 SDMMC uses internal IDMA,
// not a DMA1/DMA2 channel. See sdlog.rs.
bind_interrupts!(pub struct SdmmcIrq {
    SDMMC1 => embassy_stm32::sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

// RC receiver (USART6, RX only, 420 kbps CRSF) -- DMA2_CH2. See rc_kill.rs.
bind_interrupts!(pub struct RcIrqs {
    USART6       => embassy_stm32::usart::InterruptHandler<peripherals::USART6>;
    DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
});

// Phoenix Pi-companion MAVLink telemetry link (USART2, full duplex) --
// DMA2_CH5 (TX), DMA2_CH6 (RX). See phoenix_telem.rs.
bind_interrupts!(pub struct TelemIrqs {
    USART2       => embassy_stm32::usart::InterruptHandler<peripherals::USART2>;
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH5>;
    DMA2_STREAM6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH6>;
});

// ----------------------------------------------------------
// -------------------- IMU (SPI2 / BMI088) -----------------
// ----------------------------------------------------------

type Spi2Bus = Mutex<NoopRawMutex, Spi<'static, Async, SpiMaster>>;

static SPI2_BUS: StaticCell<Spi2Bus> = StaticCell::new();

#[embassy_executor::task]
pub async fn imu_reader_task(r: ImuResources) -> ! {
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(8_000_000);
    spi_cfg.mode = spi::MODE_3;
    spi_cfg.miso_pull = Pull::Up;

    let spi = Spi::new(
        r.spi, r.sclk, r.mosi, r.miso, r.dma_tx, r.dma_rx, Spi2Irqs, spi_cfg,
    );

    let bus = SPI2_BUS.init(Mutex::new(spi));
    let cs_acc = Output::new(r.cs_acc, Level::High, Speed::High);
    let cs_gyr = Output::new(r.cs_gyr, Level::High, Speed::High);
    let accel_dev = SpiDeviceWithConfig::new(bus, cs_acc, spi_cfg);
    let gyro_dev = SpiDeviceWithConfig::new(bus, cs_gyr, spi_cfg);

    let inner = Bmi088::new(accel_dev, gyro_dev);
    let mut imu = Bmi088Imu6Dof::new(inner);

    let mut attempt = 0u32;
    loop {
        match imu.init().await {
            Ok(()) => {
                defmt::info!("[imu_reader] BMI088 initialized");
                crate::log::log("[imu] BMI088 init OK");
                break;
            }
            Err(e) => {
                defmt::error!("[imu_reader] BMI088 init failed: {:?}", e);
                attempt += 1;
                match attempt {
                    1 => crate::log::log("[imu] BMI088 init FAIL attempt 1"),
                    2 => crate::log::log("[imu] BMI088 init FAIL attempt 2"),
                    3 => crate::log::log("[imu] BMI088 init FAIL attempt 3"),
                    _ => crate::log::log("[imu] BMI088 init FAIL (retrying)"),
                }
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }

    common::tasks::imu_reader::main_6dof(imu).await
}

// ----------------------------------------------------------
// -------------- Motors (TIM1 / DShot / DMA1_CH1) ----------
// ----------------------------------------------------------

use core::sync::atomic::{AtomicU16, Ordering as AtOrd};
use embassy_sync::signal::Signal;

/// Latest motor speeds written by motor_governor, read by the DShot sender.
static DSHOT_SPEEDS: [AtomicU16; 4] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];

/// Pending direction command (rare, only at startup).
static DSHOT_REVERSE: Signal<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    [bool; 4],
> = Signal::new();

/// Proxy OutputGroup that writes to atomics instead of sending DShot directly.
/// The actual DShot frames are sent by the keepalive sender at a fixed 1 kHz.
struct DshotProxy;

impl common::hw_abstraction::OutputGroup for DshotProxy {
    async fn set_motor_speeds(&mut self, speed: [u16; 4]) {
        // Remap from mixer MOTOR order to physical channel order.
        use crate::config::CHANNEL_MAP;
        for ch in 0..4 {
            DSHOT_SPEEDS[ch].store(speed[CHANNEL_MAP[ch]], AtOrd::Relaxed);
        }

        // Diagnostic: log first non-zero speed via log_critical so it reaches
        // SD / UART even when the regular channel is saturated by imu_monitor
        // at 100 Hz. Proves motor_governor has reached its 'armed loop and is
        // commanding motors independently of whatever CSV flooding is doing.
        static LOGGED_FIRST_NONZERO: core::sync::atomic::AtomicBool =
            core::sync::atomic::AtomicBool::new(false);
        if speed.iter().any(|&s| s > 0) && !LOGGED_FIRST_NONZERO.swap(true, AtOrd::Relaxed) {
            use core::fmt::Write;
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[proxy] FIRST nonzero speed [{},{},{},{}]",
                speed[0], speed[1], speed[2], speed[3]
            );
            crate::log::log_critical(s.as_str()).await;
        }
    }
    async fn set_reverse_dir(&mut self, direction: [bool; 4]) {
        use crate::config::CHANNEL_MAP;
        use core::fmt::Write;
        let ch_dir = [
            direction[CHANNEL_MAP[0]],
            direction[CHANNEL_MAP[1]],
            direction[CHANNEL_MAP[2]],
            direction[CHANNEL_MAP[3]],
        ];
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = write!(
            s,
            "[proxy] dir motor=[{},{},{},{}] ch=[{},{},{},{}]",
            direction[0] as u8,
            direction[1] as u8,
            direction[2] as u8,
            direction[3] as u8,
            ch_dir[0] as u8,
            ch_dir[1] as u8,
            ch_dir[2] as u8,
            ch_dir[3] as u8,
        );
        crate::log::log(s.as_str());
        DSHOT_REVERSE.signal(ch_dir);
    }
    async fn set_motor_speeds_min(&mut self) {
        for a in &DSHOT_SPEEDS {
            a.store(0, AtOrd::Relaxed);
        }

        // Diagnostic: log the first call via log_critical. motor_governor
        // calls this 50 times during the min-throttle arming phase (right
        // after the 1 s Timer::after). Seeing this log proves motor_governor
        // got past params::TABLE.read() at line 122 and into the arming body.
        static LOGGED_FIRST_MIN: core::sync::atomic::AtomicBool =
            core::sync::atomic::AtomicBool::new(false);
        if !LOGGED_FIRST_MIN.swap(true, AtOrd::Relaxed) {
            crate::log::log_critical("[proxy] FIRST set_motor_speeds_min call").await;
        }
    }
    async fn make_beep(&mut self) {}
}

#[embassy_executor::task]
pub async fn motor_governor_task(r: MotorResources, dshot: DshotConfig) -> ! {
    let wav = UpDmaWaveform::new(r.dma, MotorIrqs);

    // Motor pin order follows the motor_dshot.rs convention:
    //   Ch1 -> M4 (PE9), Ch2 -> M3 (PE11), Ch3 -> M2 (PE13), Ch4 -> M1 (PE14)
    let mut driver = DshotDriver::new(
        r.tim,
        r.m4, // Ch1
        r.m3, // Ch2
        r.m2, // Ch3
        r.m1, // Ch4
        wav,
        dshot as u32,
    );

    // BLHeli32/BlueJay require a dense, gap-free DShot-0 stream to recognise
    // the FC. pre_arm_loop sends one ~390 ms burst per iteration and exits as
    // soon as the arm command arrives, regardless of when the LiPo is connected.
    let mut arm_rcv = common::tasks::commander::COMMAD_ARM_VEHICLE.receiver();
    // Exit on ANY value on the arm watch, not just a standing `true`: the
    // loop only POLLS between ~1.2 s frame bursts, and the FSM's arm guard
    // (D000020) can turn an arm command into a ~20 ms true->false pulse
    // that a poll of the latest value will simply never see. D000003 and
    // D000004 sat with the whole governor+keepalive stack unstarted through
    // nine arm attempts exactly because of this. Any traffic on the watch
    // means the mission layer is alive; from then on the governor's
    // wake-based receiver catches every send, and the keepalive sender
    // takes over the dense DShot stream, so exiting early is safe.
    crate::dshot_driver::pre_arm_loop(&mut driver, || arm_rcv.try_get().is_some()).await;

    crate::log::log("[mtr] pre_arm_loop done -> keepalive sender");

    // Run motor_governor (writes speeds to DSHOT_SPEEDS via DshotProxy)
    // and the DShot keepalive sender (reads DSHOT_SPEEDS, sends DShot at 1 kHz)
    // concurrently. This ensures ESCs never see a DShot gap, regardless of
    // how slowly the control loop updates.
    embassy_futures::join::join(
        common::tasks::motor_governor::main(DshotProxy),
        dshot_keepalive_sender(driver),
    )
    .await;

    unreachable!()
}

/// Send DShot frames at ~1 kHz using the latest speeds from DSHOT_SPEEDS.
/// Handles reverse-direction commands when signalled.
async fn dshot_keepalive_sender<T, WAV>(mut driver: DshotDriver<'static, T, WAV>) -> !
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    WAV: crate::dshot_driver::WaveformGenerator<Timer = T>,
{
    use core::fmt::Write;
    let mut frame_count: u32 = 0;
    let mut dir_burst_count: u32 = 0;
    let mut beeping_prev = false;

    loop {
        // Process pending direction command (rare). BLHeli ESCs require
        // multiple consecutive direction frames (no interleaved throttle)
        // to accept a direction change. Send 10 frames at normal 1kHz rate.
        if let Some(d) = DSHOT_REVERSE.try_take() {
            dir_burst_count += 1;
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[dshot] dir#{} f={} [{},{},{},{}]",
                dir_burst_count, frame_count, d[0] as u8, d[1] as u8, d[2] as u8, d[3] as u8,
            );
            crate::log::log(s.as_str());

            for _ in 0..10 {
                driver.set_reverse_dir(d).await;
                embassy_time::Timer::after_micros(1000).await;
            }
            frame_count += 10;
        }

        // Read latest speeds and send DShot frame.
        // When all speeds are zero (arming phase), send DShot 0 (true disarm)
        // so ESCs stay stopped and accept direction commands.
        // throttle_clamp(0) would send DShot 48 (min throttle = motors spin),
        // which causes ESCs to reject direction commands.
        //
        // MOTOR_KILL overrides everything: once latched, this loop streams
        // DShot 0 no matter what DSHOT_SPEEDS holds, so a kill works even
        // with the governor task wedged (see MOTOR_KILL in dshot_driver.rs).
        // The forced zeros also make `disarmed` true below, which keeps the
        // find-my-drone beacon available after a kill.
        let speeds = if crate::dshot_driver::MOTOR_KILL.load(AtOrd::Relaxed) {
            [0u16; 4]
        } else {
            DSHOT_SPEEDS.each_ref().map(|a| a.load(AtOrd::Relaxed))
        };
        let disarmed = speeds == [0, 0, 0, 0];

        // Find-my-drone: while DISARMED and RC has been gone >30 s
        // (RC_BEACON_ACTIVE, set by rc_loss_beacon_task), pulse the DShot
        // beacon on a ~1.5 s cycle -- a short beacon burst the ESC registers,
        // then idle frames so the ~1 s tone plays and the stream stays
        // gap-free. The beacon never spins the motor. The disarmed (speeds==0)
        // gate means an in-flight RC-loss failsafe never beeps mid-air; it only
        // starts once the drone has landed and disarmed.
        const BEEP_PERIOD: u32 = 2100; // frames (~3 s at ~1.4 ms/frame)
        const BEEP_BURST: u32 = 40; // consecutive beacon frames to register
        let beeping = disarmed && crate::dshot_driver::RC_BEACON_ACTIVE.load(AtOrd::Relaxed);
        if beeping != beeping_prev {
            crate::log::log(if beeping {
                "[dshot] beacon pulsing (RC lost, disarmed)"
            } else {
                "[dshot] beacon off"
            });
            beeping_prev = beeping;
        }

        if beeping {
            if frame_count % BEEP_PERIOD < BEEP_BURST {
                driver.make_beep().await;
            } else {
                driver.set_motor_speeds_min().await;
            }
        } else if disarmed {
            driver.set_motor_speeds_min().await;
        } else {
            driver.set_motor_speeds(speeds).await;
        }
        frame_count += 1;

        // Log periodically during first 10 seconds to capture arming + direction phase.
        if frame_count % 500 == 0 && frame_count <= 10_000 {
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[dshot] f={} spd=[{},{},{},{}] (clamp 0->48)",
                frame_count, speeds[0], speeds[1], speeds[2], speeds[3],
            );
            crate::log::log(s.as_str());
        }

        embassy_time::Timer::after_micros(1000).await;
    }
}

/// Find-my-drone monitor: watches the RC link and drives RC_BEACON_ACTIVE.
/// `RC_CHANNELS` is published on every CRSF frame (~50 Hz), so a 30 s gap with
/// no new frame means the link is gone (crash, out of range, or TX off) ->
/// request the beacon; the first frame back clears it. Runs from boot, so it
/// works even on a crash-reboot before any arm. The motor-output loops turn the
/// request into an actual pulsing tone (and gate it to disarmed).
#[embassy_executor::task]
pub async fn rc_loss_beacon_task() -> ! {
    const RC_LOSS_BEACON_S: u64 = 30;
    let mut rcv = crate::rc_kill::RC_CHANNELS.receiver().unwrap();
    loop {
        let lost = embassy_time::with_timeout(
            embassy_time::Duration::from_secs(RC_LOSS_BEACON_S),
            rcv.changed(),
        )
        .await
        .is_err();
        let was = crate::dshot_driver::RC_BEACON_ACTIVE.swap(lost, AtOrd::Relaxed);
        if lost != was {
            crate::log::log(if lost {
                "[beacon] RC lost >30s -- beacon ON (find-my-drone)"
            } else {
                "[beacon] RC restored -- beacon OFF"
            });
        }
    }
}

// ----------------------------------------------------------
// ------------------- Baro (I2C2 / DPS310) -----------------
// ----------------------------------------------------------

type I2c2Bus = Mutex<
    NoopRawMutex,
    embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>,
>;
static I2C2_BUS: StaticCell<I2c2Bus> = StaticCell::new();

/// Initialise I2C2 and probe the baro address. Returns the shared bus handle
/// and the detected baro address. The baro probe also "primes" the DMA path
/// for subsequent I2C transactions -- embassy-stm32 routes empty writes
/// through a blocking (non-DMA) path, and without that first transaction the
/// next DMA transfer fails. This probe runs regardless of whether the caller
/// intends to use the baro, because it keeps the compass reads reliable.
///
/// Call site must own `BaroResources` and only call this ONCE per binary run
/// (the static cell panics on re-init).
async fn setup_i2c2(r: BaroResources) -> (&'static I2c2Bus, u8) {
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.sda_pullup = true;
    i2c_cfg.scl_pullup = true;
    i2c_cfg.frequency = Hertz(400_000);

    let i2c =
        embassy_stm32::i2c::I2c::new(r.i2c, r.scl, r.sda, r.dma_tx, r.dma_rx, I2c2Irqs, i2c_cfg);

    let bus: &'static I2c2Bus = I2C2_BUS.init(Mutex::new(i2c));

    let baro_addr = {
        use crate::dps310_i2c::{ADDR_SDO_HIGH, ADDR_SDO_LOW};
        let mut b = bus.lock().await;
        if b.write(ADDR_SDO_LOW, &[]).await.is_ok() {
            crate::log::log("[baro] SPL06 found at 0x76");
            ADDR_SDO_LOW
        } else if b.write(ADDR_SDO_HIGH, &[]).await.is_ok() {
            crate::log::log("[baro] SPL06 found at 0x77");
            ADDR_SDO_HIGH
        } else {
            crate::log::log("[baro] SPL06 not found, defaulting to 0x76");
            ADDR_SDO_LOW
        }
    };

    (bus, baro_addr)
}

/// Set up I2C2 and run altitude hold + compass reader concurrently.
///
/// DPS310/SPL06 (baro, 0x76/0x77) and QMC5883L (compass, 0x0D) share I2C2
/// via a NoopRawMutex shared-bus wrapper. Both devices run in the same task
/// so there is no cross-executor contention; NoopRawMutex is safe here.
#[embassy_executor::task]
pub async fn alt_hold_task(r: BaroResources) -> ! {
    let (bus, baro_addr) = setup_i2c2(r).await;

    embassy_futures::join::join(
        crate::alt_hold::main(I2cDevice::new(bus), baro_addr),
        compass_reader(I2cDevice::new(bus)),
    )
    .await;

    unreachable!()
}

/// Set up I2C2 and run ONLY the compass reader.
///
/// Used by test binaries (flight, sub_hover_test, pid_sweep_test) that
/// want magnetometer yaw fusion in att_estimator but deliberately skip
/// altitude-hold (because the mission owns TRUE_Z_THRUST_SP directly and
/// running alt_hold concurrently creates a setpoint race).
#[embassy_executor::task]
pub async fn compass_only_task(r: BaroResources) -> ! {
    let (bus, _baro_addr) = setup_i2c2(r).await;
    compass_reader(I2cDevice::new(bus)).await
}

/// Read QMC5883L magnetometer at ~50 Hz and publish to att_estimator.
///
/// The Madgwick filter in att_estimator uses this for absolute yaw reference,
/// eliminating the yaw drift that causes oscillation without a compass.
async fn compass_reader(i2c: impl embedded_hal_async::i2c::I2c) -> ! {
    use crate::qmc5883l::Qmc5883l;

    /// QMC5883L sensitivity at 2G range: 0.244 uT/LSB.
    const SENSITIVITY_UT_PER_LSB: f32 = 0.244;

    let mut compass = Qmc5883l::new(i2c);
    loop {
        match compass.init().await {
            Ok(()) => {
                crate::log::log("[compass] QMC5883L init OK");
                crate::log::SENSORS_READY.fetch_or(
                    crate::log::SENSOR_COMPASS_BIT,
                    core::sync::atomic::Ordering::Release,
                );
                break;
            }
            Err(_) => {
                crate::log::log("[compass] QMC5883L init FAIL (retrying)");
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }

    let mut snd = common::signals::CAL_MULTI_MAG_DATA[0].sender();
    loop {
        if let Ok(d) = compass.read().await {
            let centered = [
                d.x as f32 * SENSITIVITY_UT_PER_LSB - MAG_BIAS_UT[0],
                d.y as f32 * SENSITIVITY_UT_PER_LSB - MAG_BIAS_UT[1],
                d.z as f32 * SENSITIVITY_UT_PER_LSB - MAG_BIAS_UT[2],
            ];
            snd.send([
                MAG_CAL_MAT[0][0] * centered[0]
                    + MAG_CAL_MAT[0][1] * centered[1]
                    + MAG_CAL_MAT[0][2] * centered[2],
                MAG_CAL_MAT[1][0] * centered[0]
                    + MAG_CAL_MAT[1][1] * centered[1]
                    + MAG_CAL_MAT[1][2] * centered[2],
                MAG_CAL_MAT[2][0] * centered[0]
                    + MAG_CAL_MAT[2][1] * centered[1]
                    + MAG_CAL_MAT[2][2] * centered[2],
            ]);
        }
        embassy_time::Timer::after_millis(20).await;
    }
}
