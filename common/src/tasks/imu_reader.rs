use embassy_futures::select::select;
use embedded_hal_async::spi::SpiDevice;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use portable_atomic::{AtomicUsize, Ordering};

use crate::signals::register_error;
use crate::sync::channel::Channel;
use crate::types::measurements::Imu6DofData;
use crate::{drivers::imu::ImuConfig, hw_abstraction::Imu6Dof};
use crate::{get_ctrl_freq, signals as s, NUM_IMU};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_async::i2c::I2c;

static IMU_IDX: AtomicUsize = AtomicUsize::new(0);

pub mod params {
    use crate::{
        calibration::sens3d::Calib3D, tasks::param_storage::Table, utils::rot_matrix::Rotation,
    };

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        pub rot: Rotation,
        #[param(rename = "cal_a")]
        pub cal_acc: Calib3D,
        #[param(rename = "cal_g")]
        pub cal_gyr: Calib3D,
    }

    crate::const_default!(
        Params => {
            rot: Rotation::const_default(),
            cal_acc: Calib3D::const_default(),
            cal_gyr: Calib3D::const_default(),
        }
    );

    pub static TABLE: Table<Params> = Table::new("imu", Params::const_default());
}

pub enum Message {
    ReloadParams,
}

pub static CHANNEL: [Channel<Message, 1, CriticalSectionRawMutex>; NUM_IMU] = {
    const CONST_INIT: Channel<Message, 1, CriticalSectionRawMutex> = Channel::new();
    [CONST_INIT; NUM_IMU]
};

pub async fn main_6dof_i2c(mut i2c: impl I2c, config: ImuConfig, addr: Option<u8>) -> ! {
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

pub async fn main_6dof(mut imu: impl Imu6Dof) -> ! {
    const ID: &str = "imu_reader_6dof";
    info!("{}: Task started", ID);

    let idx = IMU_IDX.fetch_add(1, Ordering::AcqRel);
    assert!(idx < NUM_IMU, "Invalid index for IMU reader task");

    // Task inputs
    let rcv_messages = CHANNEL[idx].receiver();

    // Task outputs
    let mut snd_raw_imu_data = s::RAW_MULTI_IMU_DATA[idx].sender();
    let mut snd_cal_imu_data = s::CAL_MULTI_IMU_DATA[idx].sender();

    // Wait for initial configuration values
    let params = params::TABLE.read_initialized().await;

    let mut acc_cal = params.cal_acc;
    let mut gyr_cal = params.cal_gyr;
    let mut imu_rot = params.rot;

    drop(params);

    // Sampling time
    let mut ticker = Ticker::every(Duration::from_hz(get_ctrl_freq!() as u64));

    info!("{}: Entering main loop", ID);
    'infinite: loop {
        // Tighter loop for when we are in flight
        match select(rcv_messages.receive(), ticker.next()).await {
            embassy_futures::select::Either::First(message) => match message {
                Message::ReloadParams => {
                    debug!("[{}] Reloading parameters", ID);
                    let intrinsics = params::TABLE.read_initialized().await;

                    acc_cal = intrinsics.cal_acc;
                    gyr_cal = intrinsics.cal_gyr;
                    imu_rot = intrinsics.rot;
                }
            },
            embassy_futures::select::Either::Second(()) => {
                // Read the IMU data (accel and gyro)
                let raw_imu_data = match imu.read_acc_gyr().await {
                    Ok(raw_imu_data) => raw_imu_data,
                    Err(error) => {
                        error!("{}: Failed to read IMU data: {:?}", ID, error);
                        register_error(error);
                        continue 'infinite;
                    }
                };

                let timestamp = Instant::now();

                // Apply rotation
                let rot_acc_data = &imu_rot * raw_imu_data.acc.into();
                let rot_gyr_data = &imu_rot * raw_imu_data.gyr.into();

                // Rotated RAW struct
                let rot_imu_data = Imu6DofData {
                    timestamp_us: timestamp.as_micros(),
                    acc: rot_acc_data.into(),
                    gyr: rot_gyr_data.into(),
                };

                // Apply offset and scale
                let cal_acc_data = acc_cal.apply(rot_acc_data);
                let cal_gyr_data = gyr_cal.apply(rot_gyr_data);

                // Calibrated struct
                let cal_imu_data = Imu6DofData {
                    timestamp_us: timestamp.as_micros(),
                    acc: cal_acc_data.into(),
                    gyr: cal_gyr_data.into(),
                };

                // Transmit
                critical_section::with(|_| {
                    snd_raw_imu_data.send(rot_imu_data);
                    snd_cal_imu_data.send(cal_imu_data);
                });
            }
        }
    }
}

/*

pub async fn main_9dof_i2c(mut i2c: impl I2c, config: &ImuConfig, addr: Option<u8>) -> ! {
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

/// Not finished
pub async fn main_9dof(mut marg: impl Imu9Dof) -> ! {
    const ID: &str = "imu_reader_9dof";
    info!("{}: Task started", ID);

    let idx = IMU_IDX.fetch_add(1, Ordering::AcqRel);
    assert!(idx < NUM_IMU, "Invalid index for IMU reader task");

    // Task inputs
    let mut rcv_cfg_acc_cal = s::CFG_MULTI_ACC_CAL[idx].receiver();
    let mut rcv_cfg_gyr_cal = s::CFG_MULTI_GYR_CAL[idx].receiver();
    let mut rcv_cfg_mag_cal = s::CFG_MULTI_MAG_CAL[idx].receiver();
    let mut rcv_cfg_imu_rot = s::CFG_MULTI_IMU_ROT[idx].receiver();
    let mut rcv_cfg_mag_rot = s::CFG_MULTI_MAG_ROT[idx].receiver();

    // Task outputs
    let mut snd_raw_imu_data = s::RAW_MULTI_IMU_DATA[idx].sender();
    let mut snd_cal_imu_data = s::CAL_MULTI_IMU_DATA[idx].sender();
    let mut snd_raw_mag_data = s::RAW_MULTI_MAG_DATA[idx].sender();
    let mut snd_cal_mag_data = s::CAL_MULTI_MAG_DATA[idx].sender();

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
        rcv_cfg_acc_cal
            .try_changed()
            .map(|new_acc_cal: _| acc_cal = new_acc_cal);
        rcv_cfg_gyr_cal
            .try_changed()
            .map(|new_gyr_cal: _| gyr_cal = new_gyr_cal);
        rcv_cfg_imu_rot
            .try_changed()
            .map(|new_imu_rot: _| imu_rot = new_imu_rot);
        rcv_cfg_mag_cal
            .try_changed()
            .map(|new_mag_cal: _| mag_cal = new_mag_cal);
        rcv_cfg_mag_rot
            .try_changed()
            .map(|new_mag_rot: _| mag_rot = new_mag_rot);

        // Tighter loop for when we are in flight
        let mut counter = 0;
        'hyper: loop {
            ticker.next().await;

            let imu_6dof_data: Imu6DofData<f32>;
            let mut mag_data: Option<[f32; 3]> = None;

            // Only read the magnetometer every ANGLE_LOOP_DIV iterations
            if counter % ANGLE_LOOP_DIV == 0 {
                // Read the MARG data (accel, gyro, and mag)
                match marg.read_acc_gyr_mag().await {
                    Ok(raw_imu_data) => {
                        imu_6dof_data = raw_imu_data.into();
                        mag_data = Some(raw_imu_data.mag);
                    }
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
                    }
                    Err(error) => {
                        error!("{}: Failed to read IMU data: {:?}", ID, error);
                        register_error(error);
                        continue 'hyper;
                    }
                };
            }

            counter = counter.wrapping_add(1);

            let timestamp = Instant::now();

            // Apply rotation
            let rot_acc_data = &imu_rot * imu_6dof_data.acc.into();
            let rot_gyr_data = &imu_rot * imu_6dof_data.gyr.into();

            // Apply offset and scale
            let cal_acc_data = acc_cal.apply(rot_acc_data);
            let cal_gyr_data = gyr_cal.apply(rot_gyr_data);

            // Rotated RAW struct
            let rot_imu_data = Imu6DofData {
                timestamp_us: timestamp.as_micros(),
                acc: rot_acc_data.into(),
                gyr: rot_gyr_data.into(),
            };

            // Calibrated struct
            let cal_imu_data = Imu6DofData {
                timestamp_us: timestamp.as_micros(),
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

*/
