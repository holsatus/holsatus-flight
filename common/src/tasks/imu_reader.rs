use embedded_hal_async::spi::SpiDevice;
use portable_atomic::{AtomicUsize, Ordering};

use crate::types::measurements::Imu6DofData;
use crate::{
    drivers::imu::ImuConfig,
    hw_abstraction::{Imu6Dof, Imu9Dof},
};
use crate::{get_ctrl_freq, get_or_warn, signals as s, NUM_IMU};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c;

static IMU_IDX: AtomicUsize = AtomicUsize::new(0);

pub async fn main_6dof_i2c(
    mut i2c: impl I2c,
    config: &ImuConfig,
    addr: Option<u8>,  
) -> ! {
    const ID: &str = "imu_setup_6dof_i2c";
    
    let imu = loop {
        match config.i2c_setup_6dof(&mut i2c, addr).await {
            Ok(imu) => break imu,
            Err(error) => {
                error!("{}: Setup of I2C imu failed: {:?}", ID, error);
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
                Timer::after_secs(1).await;
            }
        };
    };

    main_6dof(imu).await
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
        if let Some(new_acc_cal) = rcv_cfg_acc_cal.try_changed() {
            acc_cal = new_acc_cal;
        }
        if let Some(new_gyr_cal) = rcv_cfg_gyr_cal.try_changed() {
            gyr_cal = new_gyr_cal;
        }
        if let Some(new_imu_rot) = rcv_cfg_imu_rot.try_changed() {
            imu_rot = new_imu_rot;
        }

        // Tighter loop for when we are in flight
        'hyper: loop {

            ticker.next().await;

            // Read the IMU data (accel and gyro)
            let Ok(raw_imu_data) = imu.read_acc_gyr().await else {
                error!("{}: Failed to read IMU data", ID);
                continue 'hyper;
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
pub async fn main_9dof(mut marg: impl Imu9Dof, period: Duration) -> ! {
    const ID: &str = "imu_reader_9dof";
    info!("{}: Task started", ID);
    let snd_marg_data = crate::signals::RAW_MAG_DATA.sender();

    let mut ticker = Ticker::every(period);

    info!("{}: Entering main loop", ID);
    loop {
        ticker.next().await;
        let Ok(marg_data) = marg.read_acc_gyr_mag().await else {
            error!("{}: Failed to read MARG data", ID);
            ticker.reset_after(Duration::from_millis(50));
            continue;
        };

        trace!("marg data: {:?}", marg_data);
        snd_marg_data.send(marg_data);
    }
}
