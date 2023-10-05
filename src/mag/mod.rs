use crate::{channels, config::definitions::{SimpleCalibration, Configuration}};

use defmt::*;
use embassy_time::{Ticker, Duration};

static TASK_ID : &str = "[MAG_MASTER]";

#[embassy_executor::task]
pub async fn mag_master(
    config: &'static Configuration,
    mut s_do_mag_cal: channels::DoMagCalSub,
    p_mag_reading: channels::MagReadingPub,
    p_mag0_features: channels::Mag0FeaturesPub,
) {

    #[cfg(feature = "icm20948-async")]
    let mut sub_icm20948 = unwrap!(
        crate::drivers::task_icm20948_driver::ICM20948_MAG_READING.subscriber()
    );

    let mag0_config = config.mag0.unwrap_or_default();
    
    info!("{}: Entering main loop",TASK_ID);
    loop {

        #[cfg(feature = "icm20948-async")]
        let mut reading = sub_icm20948.next_message_pure().await;

        // Apply extrinsics configuration
        mag0_config.mag_ext.translate(&mut reading);

        // Apply calibration if it is available
        if let Some(cal) = mag0_config.mag_cal {
            reading.zip_zip_apply(&cal.scale, &cal.offset, |m,o,s| {
                *m = ( *m - o ) / s
            });
        }

        p_mag_reading.publish_immediate(reading);

        // Do gyroscope calibration
        if Some(true) == s_do_mag_cal.try_next_message_pure() {

            let mut mag0_config = config.mag0.clone().unwrap_or_default();

            use mag_calibrator_rs::MagCalibrator;
            let mut mag_cal = MagCalibrator::<30>::new().pre_scaler(500.);
            let mut ticker = Ticker::every(Duration::from_hz(50));
            loop {
                let mut reading = sub_icm20948.next_message_pure().await;
                mag0_config.mag_ext.translate(&mut reading);
                mag_cal.evaluate_sample_vec(reading);
                info!("MSD : {}", mag_cal.get_mean_distance());
                if mag_cal.get_mean_distance() > 0.02 {
                    if let Some(cal) = mag_cal.perform_calibration() {
                        info!("CAL_SCALE:  {}",Debug2Format(&cal.scale));
                        info!("CAL_OFFSET: {}",Debug2Format(&cal.offset));
                        mag0_config.mag_cal = Some(SimpleCalibration { scale: cal.scale, offset: cal.offset });
                        p_mag0_features.publish_immediate(mag0_config);
                        break;
                    }
                }
                ticker.next().await;
            }
        }
    }
}
