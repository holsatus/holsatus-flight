use embassy_futures::select::select;
use embassy_time::Timer;
use futures::TryFutureExt;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use portable_atomic::{AtomicUsize, Ordering};

use crate::calibration::sens3d::Calib3D;
use crate::drivers::imu::trigger::Trigger;
use crate::drivers::imu::{ImuInitialize, ImuSensor};
use crate::errors::ImuError;
use crate::sync::channel::{Channel, Receiver};
use crate::sync::watch::Sender;
use crate::tasks::imu_reader::params::Params;
use crate::tasks::param_storage::Table;
use crate::types::measurements::Imu6DofData;
use crate::utils::rot_matrix::Rotation;
use crate::{NUM_IMU, signals as s};

static SENSOR_ID: AtomicUsize = AtomicUsize::new(0);

pub mod params {
    use crate::{
        calibration::sens3d::Calib3D, tasks::param_storage::Table, utils::rot_matrix::Rotation,
    };

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

    #[cfg(feature = "imu_count_1")]
    pub static TABLE0: Table<Params> = Table::new("imu0", Params::const_default());

    #[cfg(feature = "imu_count_2")]
    pub static TABLE1: Table<Params> = Table::new("imu1", Params::const_default());

    #[cfg(feature = "imu_count_3")]
    pub static TABLE2: Table<Params> = Table::new("imu2", Params::const_default());

    #[cfg(feature = "imu_count_4")]
    pub static TABLE3: Table<Params> = Table::new("imu3", Params::const_default());
}

pub enum Message {
    ReloadParams,
}

pub static CHANNEL: [Channel<Message, 1, CriticalSectionRawMutex>; NUM_IMU] =
    [const { Channel::new() }; NUM_IMU];

const MAX_CONSECUTIVE_ERRORS: usize = 10;

pub struct ImuReader<T> {
    sensor_id: usize,
    trigger: T,
    acc_calib: Calib3D,
    gyr_calib: Calib3D,
    rotation: Rotation,
    stats: Stats,
    param_table: &'static Table<Params>,
    receiver: Receiver<'static, Message, 1, CriticalSectionRawMutex>,
    snd_raw_imu_data: Sender<'static, Imu6DofData<f32>>,
    snd_cal_imu_data: Sender<'static, Imu6DofData<f32>>,
}

#[derive(Default)]
struct Stats {
    init_errors: usize,
    total_errors: usize,
    consecutive_errors: usize,
}

impl<T: Trigger> ImuReader<T> {
    pub async fn entry<I: ImuInitialize>(
        mut interface: I::Interface,
        config: I::Config,
        trigger: T,
    ) -> ! {
        let idx = SENSOR_ID.fetch_add(1, Ordering::AcqRel);

        let param_table = match idx {
            #[cfg(feature = "imu_count_1")]
            0 => &params::TABLE0,
            #[cfg(feature = "imu_count_2")]
            1 => &params::TABLE1,
            #[cfg(feature = "imu_count_3")]
            2 => &params::TABLE2,
            #[cfg(feature = "imu_count_4")]
            3 => &params::TABLE3,
            _ => panic!("Invalid IMU index"),
        };

        let mut runner = ImuReader {
            sensor_id: idx,
            trigger,
            acc_calib: Calib3D::const_default(),
            gyr_calib: Calib3D::const_default(),
            rotation: Rotation::const_default(),
            stats: Stats::default(),
            param_table,
            receiver: CHANNEL[idx].receiver(),
            snd_raw_imu_data: s::RAW_MULTI_IMU_DATA[idx].sender(),
            snd_cal_imu_data: s::CAL_MULTI_IMU_DATA[idx].sender(),
        };

        'setup: loop {
            let mut sensor = match I::initialize(&mut interface, &config).await {
                Ok(sensor) => sensor,
                Err(error) => {
                    error!(
                        "[imu_reader:{}] Error during initialization: {:?}",
                        runner.sensor_id, error
                    );

                    // TODO: Register error globally
                    Timer::after_millis(500).await;
                    runner.stats.init_errors += 1;
                    runner.stats.consecutive_errors += 1;
                    continue 'setup;
                }
            };

            runner.reload_parameters().await;
            runner.run_inner(&mut sensor).await;
        }
    }
}

impl<T: Trigger> ImuReader<T> {
    async fn run_inner<S: ImuSensor>(&mut self, sensor: &mut S) {
        loop {
            match select(self.receiver.receive(), self.trigger.next_trigger()).await {
                embassy_futures::select::Either::First(message) => match message {
                    Message::ReloadParams => self.reload_parameters().await,
                },
                embassy_futures::select::Either::Second(()) => {
                    if let Err(error) = self.on_trigger(sensor).await {
                        debug!(
                            "[imu_reader:{}] Too many consecutive errors, reinitializing sensor: {:?}",
                            self.sensor_id, error
                        );
                        return;
                    }
                }
            }
        }
    }

    async fn on_trigger<S: ImuSensor>(&mut self, sensor: &mut S) -> Result<(), ()> {
        match self.read_sensor(sensor).await {
            Ok(_) => {
                self.stats.consecutive_errors = 0;
                Ok(())
            }
            Err(error) => {
                debug!("[imu_reader:{}] Error: {:?}", self.sensor_id, error);
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
        debug!("[imu_reader:{}] Reloading parameters", self.sensor_id);
        let intrinsics = self.param_table.read().await;

        self.acc_calib = intrinsics.acc_cal;
        self.gyr_calib = intrinsics.gyr_cal;
        self.rotation = intrinsics.rot;
    }

    fn read_sensor<S: ImuSensor>(
        &mut self,
        sensor: &mut S,
    ) -> impl Future<Output = Result<(), ImuError>> {
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
