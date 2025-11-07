use embassy_futures::select::{select, Either};

use crate::{
    calibration::Feedback,
    calibration::{
        acc_routine::calibrate_acc,
        gyr_routine::calibrate_gyr_bias,
        mag_routine::{calibrate_mag, MagCalState},
        Calibrate,
    },
    signals::{self as s, register_error},
    tasks::param_storage,
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
            Calibrate::Acc(acc_calib, _) => {
                let idx = 0;
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Acc));
                match calibrate_acc(acc_calib, idx).await {
                    Ok(calibration) => {
                        use crate::tasks::imu_reader::{params::TABLE, Message, CHANNEL};
                        
                        // TODO: This is hacky. Subsystems should not modify parameter tables directly
                        let mut table = TABLE.params.write().await;
                        table.cal_acc = calibration;
                        info!("[{}] Setting acc calib: {:?}", ID, table.cal_acc);
                        drop(table);

                        param_storage::send(param_storage::Request::SaveTable(TABLE.name))
                            .await;

                        if let Some(channel) = CHANNEL.get(idx as usize) {
                            channel.send(Message::ReloadParams).await;
                        }
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: ACC calibration failed: {:?}", ID, error);
                    }
                }
            }
            // Calibrate gyroscope
            Calibrate::Gyr(gyr_calib, _) => {
                let idx = 0;
                snd_calibrator_state.send(CalibratorState::Calibrating(Sensor::Gyr));
                match calibrate_gyr_bias(gyr_calib, idx).await {
                    Ok(calibration_bias) => {
                        use crate::tasks::imu_reader::{params::TABLE, Message, CHANNEL};
                        
                        // TODO: This is hacky. Subsystems should not modify parameter tables directly
                        let mut table = TABLE.params.write().await;
                        table.cal_gyr.bias = calibration_bias.into();
                        info!("[{}] Setting gyr calib: {:?}", ID, table.cal_gyr);
                        drop(table);

                        param_storage::send(param_storage::Request::SaveTable(TABLE.name))
                            .await;

                        if let Some(channel) = CHANNEL.get(idx as usize) {
                            channel.send(Message::ReloadParams).await;
                        }
                    }
                    Err(error) => {
                        register_error(error);
                        warn!("{}: GYR calibration failed: {:?}", ID, error);
                    }
                }
            }
            // Calibrate magnetometer
            Calibrate::Mag(mag_calib, _) => {
                let idx = 0;
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
        }
    }
}
