use embassy_executor::SendSpawner;
use embassy_futures::select::select;
use embassy_time::Timer;
use futures::TryFutureExt;

use crate::abstraction::imu::{AccelGyro, ImuInitialize};
use crate::abstraction::trigger::Trigger;
use crate::calibration::sens3d::Calib3D;
use crate::errors::SensorError;
use crate::params::ParamTable;
use crate::sync::channel::Channel;
use crate::sync::{channel, watch};
use crate::types::measurements::Imu6DofData;
use crate::utils::rot_matrix::Rotation;
use crate::{IMU_COUNT, ImuIndex, signals as s};

pub mod params {
    use crate::params::ParamTable;
    use crate::{calibration::sens3d::Calib3D, utils::rot_matrix::Rotation};

    #[derive(Clone, Debug, mav_param::Tree)]
    pub struct Params {
        pub rot: Rotation,
        #[param(rename = "acal")]
        pub acc_cal: Calib3D,
        #[param(rename = "gcal")]
        pub gyr_cal: Calib3D,
    }

    crate::const_default!(
        Params => {
            rot: Rotation::const_default(),
            acc_cal: Calib3D::const_default(),
            gyr_cal: Calib3D::const_default(),
        }
    );

    pub static TABLES: [ParamTable<Params>; crate::IMU_COUNT] = [
        #[cfg(feature = "imu_count_1")]
        ParamTable::default("imu0"),
        #[cfg(feature = "imu_count_2")]
        ParamTable::default("imu1"),
        #[cfg(feature = "imu_count_3")]
        ParamTable::default("imu2"),
        #[cfg(feature = "imu_count_4")]
        ParamTable::default("imu3"),
    ];
}

#[embassy_executor::task(pool_size = IMU_COUNT)]
async fn params_notifier(imu_index: crate::ImuIndex) -> ! {
    params::TABLES[imu_index as usize]
        .run_notifier(|| CHANNEL[imu_index as usize].send(Message::ReloadParams))
        .await
}

pub enum Message {
    ReloadParams,
}

pub static CHANNEL: [Channel<Message, 1>; IMU_COUNT] = [const { Channel::new() }; IMU_COUNT];

const MAX_CONSECUTIVE_ERRORS: usize = 10;

pub struct ImuReader<'a, T> {
    imu_index: ImuIndex,
    trigger: T,
    stats: Stats,
    acc_calib: Calib3D,
    gyr_calib: Calib3D,
    rotation: Rotation,
    recv_channel: channel::Receiver<'a, Message, 1>,
    snd_raw_imu_data: watch::Sender<'a, Imu6DofData<f32>>,
    snd_cal_imu_data: watch::Sender<'a, Imu6DofData<f32>>,
    param_table: &'static ParamTable<params::Params>,
}

#[derive(Default)]
struct Stats {
    init_errors: usize,
    total_errors: usize,
    consecutive_errors: usize,
}

impl<T: Trigger> ImuReader<'_, T> {
    pub async fn entry<I: ImuInitialize>(
        imu_index: crate::ImuIndex,
        mut interface: I::Interface,
        config: I::Config,
        trigger: T,
    ) -> ! {
        if let Ok(task) = params_notifier(imu_index) {
            SendSpawner::for_current_executor().await.spawn(task);
        }

        let mut runner = ImuReader {
            imu_index,
            trigger,
            stats: Stats::default(),
            acc_calib: Calib3D::const_default(),
            gyr_calib: Calib3D::const_default(),
            rotation: Rotation::const_default(),
            param_table: &params::TABLES[imu_index as usize],
            recv_channel: CHANNEL[imu_index as usize].receiver(),
            snd_raw_imu_data: s::RAW_MULTI_IMU_DATA[imu_index as usize].sender(),
            snd_cal_imu_data: s::CAL_MULTI_IMU_DATA[imu_index as usize].sender(),
        };

        'setup: loop {
            let mut sensor = match I::initialize(&mut interface, &config).await {
                Ok(sensor) => sensor,
                Err(error) => {
                    error!(
                        "[imu_reader:{}] Error during initialization: {:?}",
                        runner.imu_index as u8, error
                    );

                    // TODO: Register error globally
                    Timer::after_millis(500).await;
                    runner.stats.init_errors += 1;
                    runner.stats.consecutive_errors += 1;
                    continue 'setup;
                }
            };

            runner.reload_parameters().await;

            // Returns on too many consecutive errors, loop back and re-initialize
            runner.run_inner(&mut sensor).await;
        }
    }
}

impl<T: Trigger> ImuReader<'_, T> {
    async fn run_inner<S: AccelGyro>(&mut self, sensor: &mut S) {
        loop {
            match select(self.recv_channel.receive(), self.trigger.next_trigger()).await {
                embassy_futures::select::Either::First(message) => match message {
                    Message::ReloadParams => self.reload_parameters().await,
                },
                embassy_futures::select::Either::Second(()) => {
                    if let Err(error) = self.on_trigger(sensor).await {
                        debug!(
                            "[imu_reader:{}] Too many consecutive errors, reinitializing sensor: {:?}",
                            self.imu_index as u8, error
                        );
                        return;
                    }
                }
            }
        }
    }

    async fn on_trigger<S: AccelGyro>(&mut self, sensor: &mut S) -> Result<(), ()> {
        match self.read_sensor(sensor).await {
            Ok(_) => {
                self.stats.consecutive_errors = 0;
                Ok(())
            }
            Err(error) => {
                debug!("[imu_reader:{}] Error: {:?}", self.imu_index as u8, error);
                self.stats.total_errors += 1;
                if self.stats.consecutive_errors < MAX_CONSECUTIVE_ERRORS {
                    self.stats.consecutive_errors += 1;
                    Ok(())
                } else {
                    Err(())
                }
            }
        }
    }

    async fn reload_parameters(&mut self) {
        debug!("[imu_reader:{}] Reloading parameters", self.imu_index as u8);
        let params = self.param_table.read().await;

        self.acc_calib = params.acc_cal;
        self.gyr_calib = params.gyr_cal;
        self.rotation = params.rot;
    }

    fn read_sensor<S: AccelGyro>(
        &mut self,
        sensor: &mut S,
    ) -> impl Future<Output = Result<(), SensorError>> {
        sensor
            .read_acc_gyr()
            .map_ok(|raw_imu_data| self.on_imu_data(raw_imu_data))
    }

    fn on_imu_data(&mut self, raw_imu_data: Imu6DofData<f32>) {
        // Apply rotation
        let rot_acc_data = &self.rotation * raw_imu_data.acc.into();
        let rot_gyr_data = &self.rotation * raw_imu_data.gyr.into();

        // Rotated RAW struct
        let rot_imu_data = Imu6DofData {
            timestamp_us: raw_imu_data.timestamp_us,
            acc: rot_acc_data.into(),
            gyr: rot_gyr_data.into(),
        };

        // Apply offset and scale
        let cal_acc_data = self.acc_calib.apply(rot_acc_data);
        let cal_gyr_data = self.gyr_calib.apply(rot_gyr_data);

        // Calibrated struct
        let cal_imu_data = Imu6DofData {
            timestamp_us: raw_imu_data.timestamp_us,
            acc: cal_acc_data.into(),
            gyr: cal_gyr_data.into(),
        };

        // Transmit
        critical_section::with(|_| {
            self.snd_raw_imu_data.send(rot_imu_data);
            self.snd_cal_imu_data.send(cal_imu_data);
        });
    }
}
