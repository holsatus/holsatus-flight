use embassy_futures::select::{select, Either};

use super::configurator::{CfgMsg, CONFIG_QUEUE};
use crate::{
    calibration::{
        acc_routine::calibrate_acc,
        gyr_routine::calibrate_gyr_bias,
        mag_routine::{calibrate_mag, MagCalState}, Calibrate
    }, filters::rate_pid::Feedback, signals::{self as s, register_error}
};

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sensor {
    Acc,
    Gyr,
    Mag(MagCalState),
}

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalibratorState {
    Idle,
    Calibrating(Sensor),
}

#[embassy_executor::task]
pub async fn main() -> ! {
    const ID: &str = "calibrator";
    let mut rcv_calibrate = unwrap!(s::CMD_CALIBRATE.receiver());
    let snd_calibrator_state = s::CALIBRATOR_STATE.sender();

    loop {
        snd_calibrator_state.send(CalibratorState::Idle);
        match rcv_calibrate.changed().await {

            // Calibrate accelerometer
            Calibrate::Acc((acc_calib, Some(idx))) => {
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Acc));
                match calibrate_acc(acc_calib, idx).await {
                    Ok(calibration) => {
                        CONFIG_QUEUE.send(CfgMsg::SetAccCalib((calibration, idx))).await;
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: ACC calibration failed: {:?}", ID, error);
                    }
                }
            }
            Calibrate::Acc((_acc_calib, None)) => {
                error!("Simultaneous accelerometer calibration not yet supported")
            }

            // Calibrate gyroscope
            Calibrate::Gyr((gyr_calib, Some(idx))) => {
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Gyr));
                match calibrate_gyr_bias(gyr_calib, idx).await {
                    Ok(calibration_bias) => {
                        let mut calibration = s::CFG_MULTI_GYR_CAL[idx as usize]
                            .try_get().unwrap_or_default();
                        calibration.set_bias(calibration_bias);
                        CONFIG_QUEUE.send(CfgMsg::SetGyrCalib((calibration, idx))).await;
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: GYR calibration failed: {:?}", ID, error);
                    }
                }
            }
            Calibrate::Gyr((_gyr_calib, None)) => {
                error!("Simultaneous gyro calibration not yet supported")
            }

            // Calibrate magnetometer
            Calibrate::Mag((mag_calib, Some(idx))) => {
                let feedback = Feedback::new();
                match select(
                    calibrate_mag(mag_calib, idx, feedback.handle()),
                    async { while let Some(state) = feedback.receive().await {
                        snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Mag(state)));
                    }
                }).await {
                    Either::First(Ok(calibration)) => {
                        CONFIG_QUEUE.send(CfgMsg::SetMagCalib((calibration, idx))).await;
                    }
                    Either::First(Err(error)) => {
                        register_error(error);
                        warn!("{}: GYR calibration failed: {:?}", ID, error);
                    }
                    Either::Second(_) => {
                        error!("Calibration feedback signal was dropped")
                    }
                }
            }
            Calibrate::Mag((_mag_calib, None)) => {
                error!("Simultaneous gyro calibration not yet supported")
            }
        }
    }
}
