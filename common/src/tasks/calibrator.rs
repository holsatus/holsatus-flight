use embassy_futures::select::{select, Either};

use crate::{
    calibration::{
        acc_routine::calibrate_acc,
        gyr_routine::calibrate_gyr_bias,
        mag_routine::{calibrate_mag, MagCalState},
        Calibrate,
    },
    filters::rate_pid::Feedback,
    signals::{self as s, register_error},
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
    let mut rcv_calibrate = s::CMD_CALIBRATE.receiver();
    let mut snd_calibrator_state = s::CALIBRATOR_STATE.sender();

    loop {
        snd_calibrator_state.send(CalibratorState::Idle);
        match rcv_calibrate.changed().await {
            // Calibrate accelerometer
            Calibrate::Acc(acc_calib, Some(idx)) => {
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Acc));
                match calibrate_acc(acc_calib, idx).await {
                    Ok(calibration) => {
                        use crate::tasks::imu_reader::{Message, CHANNEL};
                        if let Some(channel) = CHANNEL.get(idx as usize) {
                            channel.send(Message::SetAccCalib(calibration)).await;
                        }
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: ACC calibration failed: {:?}", ID, error);
                    }
                }
            }
            Calibrate::Acc(_acc_calib, None) => {
                error!("Simultaneous accelerometer calibration not yet supported")
            }

            // Calibrate gyroscope
            Calibrate::Gyr(gyr_calib, Some(idx)) => {
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Gyr));
                match calibrate_gyr_bias(gyr_calib, idx).await {
                    Ok(calibration_bias) => {
                        use crate::calibration::sens3d::{Calib3DType, SmallCalib3D};
                        use crate::tasks::imu_reader::{Message, CHANNEL};
                        if let Some(channel) = CHANNEL.get(idx as usize) {
                            channel
                                .send(Message::SetGyrCalib(Calib3DType::Small(SmallCalib3D {
                                    bias: Some(calibration_bias),
                                    scale: None,
                                })))
                                .await;
                        }
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: GYR calibration failed: {:?}", ID, error);
                    }
                }
            }
            Calibrate::Gyr(_gyr_calib, None) => {
                error!("Simultaneous gyro calibration not yet supported")
            }

            // Calibrate magnetometer
            Calibrate::Mag(mag_calib, Some(idx)) => {
                warn!("Unimplemented!");
                let feedback = Feedback::new();
                match select(calibrate_mag(mag_calib, idx, feedback.handle()), async {
                    while let Some(state) = feedback.receive().await {
                        snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Mag(state)));
                    }
                })
                .await
                {
                    Either::First(Ok(_calibration)) => {
                        warn!("Unimplemented!")
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
            Calibrate::Mag(_mag_calib, None) => {
                error!("Simultaneous gyro calibration not yet supported")
            }
        }
    }
}
