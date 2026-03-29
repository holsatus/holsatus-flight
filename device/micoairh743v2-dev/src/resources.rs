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

use assign_resources::assign_resources;

use embassy_stm32::{bind_interrupts, mode::Async, peripherals, Peri, Peripherals};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::spi::{Config as SpiConfig, mode::Master as SpiMaster, Spi};
use embassy_stm32::i2c::Config as I2cConfig;
use embassy_stm32::time::Hertz;
use embassy_stm32::spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use static_cell::StaticCell;

use common::types::config::DshotConfig;

use crate::bmi088::Bmi088;
use crate::bmi088_imu6dof::Bmi088Imu6Dof;
use crate::dshot_driver::{DshotDriver, UpDmaWaveform};

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
    leds: LedResources {
        red:   PE3,
        blue:  PE4,
        green: PE2,
    }
    uart_log: UartLogResources {
        usart: USART1,
        tx:    PA9,
        dma:   DMA1_CH0,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}

// Module-level interrupt bindings for SPI2 (DMA1_CH6 TX, DMA1_CH7 RX).
// Defined here rather than inside imu_reader_task so that standalone binaries
// (e.g. imu_data.rs) that import this crate can reuse the binding without
// introducing a duplicate #[no_mangle] interrupt handler symbol.
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
        r.spi, r.sclk, r.mosi, r.miso,
        r.dma_tx, r.dma_rx,
        Spi2Irqs,
        spi_cfg,
    );

    let bus = SPI2_BUS.init(Mutex::new(spi));
    let cs_acc = Output::new(r.cs_acc, Level::High, Speed::High);
    let cs_gyr = Output::new(r.cs_gyr, Level::High, Speed::High);
    let accel_dev = SpiDeviceWithConfig::new(bus, cs_acc, spi_cfg);
    let gyro_dev  = SpiDeviceWithConfig::new(bus, cs_gyr, spi_cfg);

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
    crate::dshot_driver::pre_arm_loop(
        &mut driver,
        || arm_rcv.try_get() == Some(true),
    ).await;

    common::tasks::motor_governor::main(driver).await
}

// ----------------------------------------------------------
// ------------------- Baro (I2C2 / DPS310) -----------------
// ----------------------------------------------------------

/// Set up I2C2 and run altitude hold + compass reader concurrently.
///
/// DPS310 (baro, 0x76) and IST8310 (compass, 0x0E) share the I2C2 bus via a
/// NoopRawMutex shared-bus wrapper. Both devices run in the same task so there
/// is no cross-executor contention; NoopRawMutex is safe here.
#[embassy_executor::task]
pub async fn alt_hold_task(r: BaroResources) -> ! {

    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.sda_pullup = true;
    i2c_cfg.scl_pullup = true;
    i2c_cfg.frequency = Hertz(400_000);

    let i2c = embassy_stm32::i2c::I2c::new(
        r.i2c, r.scl, r.sda,
        r.dma_tx, r.dma_rx,
        I2c2Irqs,
        i2c_cfg,
    );

    type I2c2Bus = Mutex<NoopRawMutex, embassy_stm32::i2c::I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>;
    static I2C2_BUS: StaticCell<I2c2Bus> = StaticCell::new();
    let bus: &'static I2c2Bus = I2C2_BUS.init(Mutex::new(i2c));

    embassy_futures::join::join(
        crate::alt_hold::main(I2cDevice::new(bus)),
        compass_reader(I2cDevice::new(bus)),
    ).await;

    unreachable!()
}

/// Read IST8310 magnetometer at ~50 Hz and publish to att_estimator.
///
/// The Madgwick filter in att_estimator uses this for absolute yaw reference,
/// eliminating the yaw drift that causes oscillation without a compass.
async fn compass_reader(i2c: impl embedded_hal_async::i2c::I2c) -> ! {
    use crate::ist8310::{Ist8310, SENSITIVITY_UT_PER_LSB};

    let mut compass = Ist8310::new(i2c);
    loop {
        match compass.init().await {
            Ok(()) => {
                crate::log::log("[compass] IST8310 init OK");
                break;
            }
            Err(_) => {
                crate::log::log("[compass] IST8310 init FAIL (retrying)");
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }

    let mut snd = common::signals::CAL_MULTI_MAG_DATA[0].sender();
    loop {
        if let Ok(d) = compass.read().await {
            snd.send([
                d.x as f32 * SENSITIVITY_UT_PER_LSB,
                d.y as f32 * SENSITIVITY_UT_PER_LSB,
                d.z as f32 * SENSITIVITY_UT_PER_LSB,
            ]);
        }
        embassy_time::Timer::after_millis(20).await;
    }
}
