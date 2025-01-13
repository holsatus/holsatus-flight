use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embedded_hal_async::spi::SpiDevice;
use portable_atomic::{AtomicUsize, Ordering};

use crate::signals::register_error;
use crate::types::measurements::Imu6DofData;
use crate::{
    drivers::imu::ImuConfig,
    hw_abstraction::{Imu6Dof, Imu9Dof},
};
use crate::{get_ctrl_freq, get_or_warn, signals as s, ANGLE_LOOP_DIV, NUM_IMU};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::i2c::I2c;

static IMU_IDX: AtomicUsize = AtomicUsize::new(0);

pub static TIME_SIG: Signal<CriticalSectionRawMutex, Instant> = Signal::new();

pub async fn main_6dof_i2c(
    mut i2c: impl I2c,
    config: &ImuConfig,
    addr: Option<u8>,  
) -> ! {
    const ID: &str = "imu_setup_6dof_i2c";
    info!("{}: Task started", ID);
    
    let imu = loop {
        match config.i2c_setup_6dof(&mut i2c, addr).await {
            Ok(imu) => break imu,
            Err(error) => {
                error!("{}: Setup of I2C imu failed: {:?}", ID, error);
                register_error(error);
                Timer::after_secs(1).await;
            }
        };
    };

    main_6dof(imu).await
}

pub async fn main_6dof_spi(
    mut i2c: impl SpiDevice<Error = embedded_hal::spi::ErrorKind>,
    config: &ImuConfig,
) -> ! {
    const ID: &str = "imu_setup_6dof_spi";
    
    let imu = loop {
        match config.spi_setup_6dof(&mut i2c).await {
            Ok(imu) => break imu,
            Err(error) => {
                error!("{}: Setup of SPI imu failed: {:?}", ID, error);
                register_error(error);
                Timer::after_secs(1).await;
            }
        };
    };

    main_6dof(imu).await
}

pub async fn main_9dof_i2c(
    mut i2c: impl I2c,
    config: &ImuConfig,
    addr: Option<u8>,  
) -> ! {
    const ID: &str = "imu_setup_9dof_i2c";
    info!("{}: Task started", ID);
    
    let imu = loop {
        match config.i2c_setup_9dof(&mut i2c, addr).await {
            Ok(imu) => break imu,
            Err(error) => {
                error!("{}: Setup of I2C imu failed: {:?}", ID, error);
                register_error(error);
                Timer::after_secs(1).await;
            }
        };
    };

    main_9dof(imu).await
}

pub async fn main_6dof(
    mut imu: impl Imu6Dof,
) -> ! {
    const ID: &str = "imu_reader_6dof";
    info!("{}: Task started", ID);

    let idx = IMU_IDX.fetch_add(1, Ordering::AcqRel);
    assert!(idx < NUM_IMU, "Invalid index for IMU reader task");

    // Task inputs
    let mut rcv_cfg_acc_cal = unwrap!(s::CFG_MULTI_ACC_CAL[idx].receiver());
    let mut rcv_cfg_gyr_cal = unwrap!(s::CFG_MULTI_GYR_CAL[idx].receiver());
    let mut rcv_cfg_imu_rot = unwrap!(s::CFG_MULTI_IMU_ROT[idx].receiver());

    // Task outputs
    let snd_raw_imu_data = s::RAW_MULTI_IMU_DATA[idx].sender();
    let snd_cal_imu_data = s::CAL_MULTI_IMU_DATA[idx].sender();

    // Wait for initial configuration values
    let mut acc_cal = get_or_warn!(rcv_cfg_acc_cal).await;
    let mut gyr_cal = get_or_warn!(rcv_cfg_gyr_cal).await;
    let mut imu_rot = get_or_warn!(rcv_cfg_imu_rot).await;

    // Sampling time
    let mut ticker = Ticker::every(Duration::from_hz(get_ctrl_freq!() as u64));

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        // Check if any of the configs have updated
        rcv_cfg_acc_cal.try_changed().map(|new_acc_cal: _| acc_cal = new_acc_cal);
        rcv_cfg_gyr_cal.try_changed().map(|new_gyr_cal: _| gyr_cal = new_gyr_cal);
        rcv_cfg_imu_rot.try_changed().map(|new_imu_rot: _| imu_rot = new_imu_rot);

        // Tighter loop for when we are in flight
        'hyper: loop {

            ticker.next().await;
            TIME_SIG.signal(Instant::now());

            // Read the IMU data (accel and gyro)
            let raw_imu_data = match imu.read_acc_gyr().await {
                Ok(raw_imu_data) => raw_imu_data,
                Err(error) => {
                    error!("{}: Failed to read IMU data: {:?}", ID, error);
                    register_error(error);
                    continue 'hyper;
                }
            };

            // Apply rotation
            let rot_acc_data = &imu_rot * raw_imu_data.acc.into();
            let rot_gyr_data = &imu_rot * raw_imu_data.gyr.into();

            // Apply offset and scale
            let cal_acc_data = acc_cal.apply(rot_acc_data);
            let cal_gyr_data = gyr_cal.apply(rot_gyr_data);

            // Rotated RAW struct
            let rot_imu_data = Imu6DofData {
                acc: rot_acc_data.into(),
                gyr: rot_gyr_data.into(),
            };

            // Calibrated struct
            let cal_imu_data = Imu6DofData {
                acc: cal_acc_data.into(),
                gyr: cal_gyr_data.into(),
            };

            // Transmit
            critical_section::with(|_| {
                snd_raw_imu_data.send(rot_imu_data);
                snd_cal_imu_data.send(cal_imu_data);
            });

            // As long as we are not in flight, we should check for updated configs
            if !s::IN_FLIGHT.load(Ordering::Relaxed) {
                continue 'infinite;
            }
        }
    }
}

/// Not finished
pub async fn main_9dof(mut marg: impl Imu9Dof) -> ! {
    const ID: &str = "imu_reader_9dof";
    info!("{}: Task started", ID);

    let idx = IMU_IDX.fetch_add(1, Ordering::AcqRel);
    assert!(idx < NUM_IMU, "Invalid index for IMU reader task");

    // Task inputs
    let mut rcv_cfg_acc_cal = unwrap!(s::CFG_MULTI_ACC_CAL[idx].receiver());
    let mut rcv_cfg_gyr_cal = unwrap!(s::CFG_MULTI_GYR_CAL[idx].receiver());
    let mut rcv_cfg_mag_cal = unwrap!(s::CFG_MULTI_MAG_CAL[idx].receiver());
    let mut rcv_cfg_imu_rot = unwrap!(s::CFG_MULTI_IMU_ROT[idx].receiver());
    let mut rcv_cfg_mag_rot = unwrap!(s::CFG_MULTI_MAG_ROT[idx].receiver());

    // Task outputs
    let snd_raw_imu_data = s::RAW_MULTI_IMU_DATA[idx].sender();
    let snd_cal_imu_data = s::CAL_MULTI_IMU_DATA[idx].sender();
    let snd_raw_mag_data = s::RAW_MULTI_MAG_DATA[idx].sender();
    let snd_cal_mag_data = s::CAL_MULTI_MAG_DATA[idx].sender();

    // Wait for initial configuration values
    let mut acc_cal = get_or_warn!(rcv_cfg_acc_cal).await;
    let mut gyr_cal = get_or_warn!(rcv_cfg_gyr_cal).await;
    let mut imu_rot = get_or_warn!(rcv_cfg_imu_rot).await;
    let mut mag_cal = get_or_warn!(rcv_cfg_mag_cal).await;
    let mut mag_rot = get_or_warn!(rcv_cfg_mag_rot).await;

    // Sampling time
    let mut ticker = Ticker::every(Duration::from_hz(get_ctrl_freq!() as u64));

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        // Check if any of the configs have updated
        rcv_cfg_acc_cal.try_changed().map(|new_acc_cal: _| acc_cal = new_acc_cal);
        rcv_cfg_gyr_cal.try_changed().map(|new_gyr_cal: _| gyr_cal = new_gyr_cal);
        rcv_cfg_imu_rot.try_changed().map(|new_imu_rot: _| imu_rot = new_imu_rot);
        rcv_cfg_mag_cal.try_changed().map(|new_mag_cal: _| mag_cal = new_mag_cal);
        rcv_cfg_mag_rot.try_changed().map(|new_mag_rot: _| mag_rot = new_mag_rot);

        // Tighter loop for when we are in flight
        let mut counter = 0;
        'hyper: loop {

            ticker.next().await;
            TIME_SIG.signal(Instant::now());

            let imu_6dof_data: Imu6DofData<f32>;
            let mut mag_data: Option<[f32; 3]> = None;

            // Only read the magnetometer every ANGLE_LOOP_DIV iterations
            if counter % ANGLE_LOOP_DIV == 0 {
                // Read the MARG data (accel, gyro, and mag)
                match marg.read_acc_gyr_mag().await {
                    Ok(raw_imu_data) => {
                        imu_6dof_data = raw_imu_data.into();
                        mag_data = Some(raw_imu_data.mag);
                    },
                    Err(error) => {
                        error!("{}: Failed to read IMU data: {:?}", ID, error);
                        register_error(error);
                        continue 'hyper;
                    }
                };
            } else {
                // Read the IMU data (accel and gyro)
                match marg.read_acc_gyr().await {
                    Ok(raw_imu_data) => {
                        imu_6dof_data = raw_imu_data;
                    },
                    Err(error) => {
                        error!("{}: Failed to read IMU data: {:?}", ID, error);
                        register_error(error);
                        continue 'hyper;
                    }
                };
            }

            counter = counter.wrapping_add(1);

            // Apply rotation
            let rot_acc_data = &imu_rot * imu_6dof_data.acc.into();
            let rot_gyr_data = &imu_rot * imu_6dof_data.gyr.into();
            
            // Apply offset and scale
            let cal_acc_data = acc_cal.apply(rot_acc_data);
            let cal_gyr_data = gyr_cal.apply(rot_gyr_data);
            
            // Rotated RAW struct
            let rot_imu_data = Imu6DofData {
                acc: rot_acc_data.into(),
                gyr: rot_gyr_data.into(),
            };
            
            // Calibrated struct
            let cal_imu_data = Imu6DofData {
                acc: cal_acc_data.into(),
                gyr: cal_gyr_data.into(),
            };

            // Apply rotation and calibration to magnetometer
            let mag_data = if let Some(mag_data) = mag_data {
                let rot_mag_data = &mag_rot * mag_data.into();
                let cal_mag_data = mag_cal.apply(rot_mag_data);
                Some((rot_mag_data, cal_mag_data))
            } else {
                None
            };

            // Transmit
            critical_section::with(|_| {
                snd_raw_imu_data.send(rot_imu_data);
                snd_cal_imu_data.send(cal_imu_data);
                if let Some((rot_mag_data, cal_mag_data)) = mag_data {
                    snd_raw_mag_data.send(rot_mag_data.into());
                    snd_cal_mag_data.send(cal_mag_data.into());
                }
            });

            // As long as we are not in flight, we should check for updated configs
            if !s::IN_FLIGHT.load(Ordering::Relaxed) {
                continue 'infinite;
            }
        }
    }
}
