use core::{hash::Hash, marker::PhantomData, ops::Range};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer, WithTimeout};
use embedded_storage_async::nor_flash::NorFlash;

use crate::{
    calibration::sens3d::Calib3DType, filters::rate_pid::RatePidCfg3D, mavlink::MavStreamCfg, multi_sender, rc_mapping::RcBindings, signals as s, sync::rpc::Procedure, types::config::{BootConfig, ImuIntrinsics, Keyed, MagIntrinsics, Vehicle, Wrap}, utils::rot_matrix::Rotation, MAIN_LOOP_FREQ, NUM_IMU, NUM_MAG
};

#[derive(Debug, Clone)]
pub enum CfgMsg {
    LoadConfig{ idx: u8 },
    SaveConfig{ idx: u8 },
    ResetConfigToDefault,
    SetAccCalib((Calib3DType, u8)),
    SetGyrCalib((Calib3DType, u8)),
    SetMagCalib((Calib3DType, u8)),
}

crate::rpc_message!{
    ConfiguratorRpc: Request -> Response

    /// Load the intrinsics for some IMU
    async GetImuInt {
        /// ID of the sensor to load intrinsics for
        id: u8
    } -> ImuIntrinsics,

    /// Load the intrinsics for some magnetometer
    async GetMagInt {
        /// ID of the sensor to load intrinsics for
        id: u8
    } -> MagIntrinsics,

    async SetAccCal {
        /// ID of the sensor to set calibration for
        id: u8,

        /// Calibration data for the sensor
        cal: Calib3DType
    } -> Res,

    async SetGyrCal { id: u8, cal: Calib3DType } -> Res,
    async SetMagCal { id: u8, cal: Calib3DType } -> Res,

    async GetRcBinds {} -> RcBindings,
    async SetRcBinds { binds: RcBindings } -> Res,
    async GetMavStreams {} -> MavStreamCfg,
    async SetMavStreams {streams: MavStreamCfg} -> Res,
    async GetRatePid {} -> RatePidCfg3D,
    async SetRatePid { cfg: RatePidCfg3D } -> Res,
    async GetAttiPid {} -> RatePidCfg3D,
    async SetAttiPid { cfg: RatePidCfg3D } -> Res,
}

pub enum Res {
    Success,
    Rejected,
}

pub static CONFIG_RPC: Procedure<ConfiguratorRpc> = Procedure::new();

pub static CONFIG_QUEUE: Channel<CriticalSectionRawMutex, CfgMsg, 5> = Channel::new();

pub async fn main(
    flash: impl NorFlash,
    flash_range: Range<u32>,
    gen_config: impl FnOnce() -> BootConfig,
) -> ! {
    const ID: &str = "configurator";
    info!("{}: Task started", ID);

    // Input signals
    let rcv_config_queue = CONFIG_QUEUE.receiver();

    // Output signals
    let mut snd_cfg_rate_loop = s::CFG_RATE_LOOP_PIDS.sender();
    let mut snd_cfg_rc_bindings = s::CFG_RC_BINDINGS.sender();
    let mut snd_cfg_control_freq = s::CFG_CONTROL_FREQ.sender();
    let mut snd_cfg_motor_dirs = s::CFG_MOTOR_DIRS.sender();
    let mut snd_cfg_vehicle_info = s::CFG_VEHICLE_INFO.sender();
    let mut snd_cfg_acc_cal = multi_sender!(s::CFG_MULTI_ACC_CAL, NUM_IMU);
    let mut snd_cfg_gyr_cal = multi_sender!(s::CFG_MULTI_GYR_CAL, NUM_IMU);
    let mut snd_cfg_imu_rot = multi_sender!(s::CFG_MULTI_IMU_ROT, NUM_IMU);
    let mut snd_cfg_mag_cal = multi_sender!(s::CFG_MULTI_MAG_CAL, NUM_MAG);
    let mut snd_cfg_mag_rot = multi_sender!(s::CFG_MULTI_MAG_ROT, NUM_MAG);

    let mut storage = Storage::new(flash, flash_range);

    // Load config from flash and initialize the once cell with the value
    match storage.read::<BootConfig>(0).await {
        Some(config) => {
            info!("{}: Boot config loaded from flash successfully", ID);
            s::BOOT_CONFIG.init(config)
        }
        None => {
            warn!("{}: Boot config not found in flash, using platform default", ID);
            s::BOOT_CONFIG.init(gen_config())
        }
    }.expect("Boot config must be set by configurator task");

    // Allow other tasks to start based on the boot config
    Timer::after_millis(10).await;

    // // Load IMU calibrations from flash
    // match storage.read::<ImuIntrinsics>(0).await {
    //     Some(calibs) => {
    //         info!("{}: IMU calibrations loaded from flash successfully", ID);
    //         snd_cfg_imu_rot.iter().zip(calibs.imu_rot).for_each(|(snd, val)|snd.send(val));
    //         snd_cfg_acc_cal.iter().zip(calibs.acc_cal).for_each(|(snd,val)|snd.send(val));
    //         snd_cfg_gyr_cal.iter().zip(calibs.gyr_cal).for_each(|(snd,val)|snd.send(val));
    //     },
    //     None => {
    //         warn!("{}: IMU calibrations not found in flash", ID);
    //     },
    // }

    // // Load Mag calibrations from flash
    // match storage.read::<MagIntrinsics>(0).await {
    //     Some(calibs) => {
    //         info!("[{}]: Mag calibrations loaded from flash successfully", ID);
    //         snd_cfg_mag_rot.iter().zip(calibs.mag_rot).for_each(|(snd, val)|snd.send(val));
    //         snd_cfg_mag_cal.iter().zip(calibs.mag_cal).for_each(|(snd,val)|snd.send(val));
    //     },
    //     None => {
    //         snd_cfg_mag_rot.iter().for_each(|snd|snd.send(Default::default()));
    //         snd_cfg_mag_cal.iter().for_each(|snd|snd.send(Default::default()));
    //         warn!("[{}]: Mag calibrations not found in flash", ID);
    //     },
    // }

    macro_rules! print_could_load {
        ($result:expr) => {
            match $result {
                Some(_) => info!("[{}]: <{}> loaded from flash successfully", ID, stringify!($result)),
                None => warn!("[{}]: <{}> not found in flash, using default", ID, stringify!($result)),
            }
        };
    }

    let rc_bindings = storage.read::<RcBindings>(0).await;
    snd_cfg_rc_bindings.send(rc_bindings.unwrap_or_default());
    print_could_load!(rc_bindings);

    // Load rate PID gains
    match storage.read::<RatePidCfg3D>(0).await {
        Some(rate_pid_cfg) => {
            info!("{}: Rate PID gains loaded from flash successfully", ID);
            snd_cfg_rate_loop.send(rate_pid_cfg)
        },
        None => {
            warn!("{}: Rate PID gains not found in flash, using defaults", ID);
            snd_cfg_rate_loop.send(RatePidCfg3D::default());
        },
    };

    snd_cfg_vehicle_info.send(storage.read::<Vehicle>(0).await.unwrap_or_default());

    match storage.read::<Vehicle>(0).await {
        Some(vehicle_info) => {
            info!("{}: Vehicle information loaded from flash successfully", ID);
            snd_cfg_vehicle_info.send(vehicle_info)
        },
        None => {
            warn!("{}: Vehicle information not found in flash, using defaults", ID);
            snd_cfg_vehicle_info.send(Vehicle::default());
        },
    };

    #[cfg(feature = "mavlink")] {
        // Request default MAVLink streams
        use crate::mavlink::{MavRequest, MavStreamCfg};

        let stream_cfg = match storage.read::<MavStreamCfg>(0).await {
            Some(vehicle_info) => {
                info!("{}: MAVLink stream loaded from flash successfully", ID);
                vehicle_info
            },
            None => {
                warn!("{}: MAVLink stream config not found in flash, using defaults", ID);
                MavStreamCfg::default()
            },
        };

        for (index, freq) in stream_cfg.streams.iter().flatten() {
            if s::MAV_REQUEST.send(MavRequest::Stream { id: *index, period_us: *freq as u32 })
                .with_timeout(Duration::from_millis(100)).await.is_err() {
                    error!("{} MAVLink server did not receive messages", ID);
                };
        }
    }

    snd_cfg_motor_dirs.send([true, false, false, true]);
    snd_cfg_control_freq.send(MAIN_LOOP_FREQ as u16);
    s::CONTROL_FREQUENCY.store(MAIN_LOOP_FREQ as u16, core::sync::atomic::Ordering::Relaxed);

    loop {

        match CONFIG_RPC.get_request().await {
            Request::GetImuInt(handle, get_imu_int) => {
                match storage.read::<ImuIntrinsics>(get_imu_int.id).await {
                    Some(mut calibs) => {
                        calibs.imu_rot = Rotation::RotX180;
                        handle.respond(calibs)
                    },
                    None => handle.close(),
                }
            },
            Request::GetMagInt(handle, get_mag_int) => {
                match storage.read::<MagIntrinsics>(get_mag_int.id).await {
                    Some(calibs) => handle.respond(calibs),
                    None => handle.close(),
                }
            },
            Request::SetAccCal(handle, set_acc_cal) => {
                let Some(mut int) = storage.read::<ImuIntrinsics>(0).await else {
                    error!("{}: Unable to read ACC calib from flash", ID);
                    handle.close();
                    continue;
                };

                int.acc_cal = set_acc_cal.cal;
                snd_cfg_acc_cal[set_acc_cal.id as usize].send(int.acc_cal);

                if storage.write(int, set_acc_cal.id).await.is_err() {
                    error!("{}: Unable to write ACC calib to flash", ID);
                }
            },
            Request::SetGyrCal(handle, set_gyr_cal) => {
                let Some(mut int) = storage.read::<ImuIntrinsics>(0).await else {
                    error!("{}: Unable to read ACC calib from flash", ID);
                    handle.respond(Res::Rejected);
                    continue;
                };

                int.acc_cal = set_gyr_cal.cal;
                snd_cfg_acc_cal[set_gyr_cal.id as usize].send(int.acc_cal);

                if storage.write(int, set_gyr_cal.id).await.is_err() {
                    error!("{}: Unable to write GYR calib to flash", ID);
                    handle.respond(Res::Rejected);
                    continue
                }

                handle.respond(Res::Success);
            },
            Request::SetMagCal(handle, set_mag_cal) => todo!(),

            Request::GetRcBinds(handle, get_rc_binds) => todo!(),
            Request::SetRcBinds(handle, set_rc_binds) => todo!(),
            Request::GetMavStreams(handle, get_mav_streams) => todo!(),
            Request::SetMavStreams(handle, set_mav_streams) => todo!(),
            Request::GetRatePid(handle, get_rate_pid) => todo!(),
            Request::SetRatePid(handle, set_rate_pid) => todo!(),
            Request::GetAttiPid(handle, get_atti_pid) => todo!(),
            Request::SetAttiPid(handle, set_atti_pid) => todo!(),
        }



        // match rcv_config_queue.receive().await {
        //     CfgMsg::LoadConfig { idx: _ } => todo!(),
        //     CfgMsg::SaveConfig { idx: _ } => todo!(),
        //     CfgMsg::ResetConfigToDefault => todo!(),
        //     CfgMsg::SetGyrCalib((new_calib, idx)) => {
        //         info!("{}: Received new GYR calib, saving to flash", ID);
        //         let mut calibs = storage.read::<ImuIntrinsics>(idx).await.unwrap_or_default();

        //         calibs.gyr_cal = new_calib;
        //         snd_cfg_gyr_cal[idx as usize].send(calibs.gyr_cal);

        //         if storage.write(calibs, 0).await.is_err() {
        //             error!("{}: Unable to write GYR calib to flash", ID);
        //         }
        //     }
        //     CfgMsg::SetMagCalib((new_calib, idx)) => {
        //         info!("{}: Received new MAG calib, saving to flash", ID);
        //         let mut calibs = storage.read::<MagIntrinsics>(idx).await.unwrap_or_default();

        //         calibs.mag_cal = new_calib;
        //         snd_cfg_mag_cal[idx as usize].send(calibs.mag_cal);

        //         if storage.write(calibs, 0).await.is_err() {
        //             error!("{}: Unable to write MAG calib to flash", ID);
        //         }
        //     }
        //     _ => {
        //         warn!("{}: Saving of this type is not yet implemented", ID);
        //     }
        // }
    }
}

pub struct Storage<NF> {
    flash: NF,
    buffer: [u8; 1024],
    range: Range<u32>,
}

enum Error {
    KeyedItemNotFound,
    FlashNotAligned,
    FlashOutOfbounds,
    FlashOther
}

impl<NF: NorFlash> Storage<NF> {
    pub fn new(flash: NF, range: Range<u32>) -> Self {
        Storage {
            flash,
            buffer: [0u8; 1024],
            range,
        }
    }

    pub async fn read<'a, I: Keyed<'a>>(&'a mut self, idx: u8) -> Option<I> {
        let search_key = (I::KEY as u16) << 8 | idx as u16;
        let result = sequential_storage::map::fetch_item::<u16, Wrap<I>, _>(
            &mut self.flash,
            self.range.clone(),
            &mut sequential_storage::cache::NoCache::new(),
            &mut self.buffer,
            &search_key,
        )
        .await;

        match result {
            Ok(Some(item)) => Some(item.0),
            _ => {
                warn!("Did not find data with key {} and index {} in flash storage", I::KEY, idx);
                None
            }
        }
    }

    pub async fn write<'a, I: Keyed<'a>>(&'a mut self, data: I, idx: u8) -> Result<(), sequential_storage::Error<NF::Error>> {
        let search_key = (I::KEY as u16) << 8 | idx as u16;
        sequential_storage::map::store_item::<u16, Wrap<I>, _>(
            &mut self.flash,
            self.range.clone(),
            &mut sequential_storage::cache::NoCache::new(),
            &mut self.buffer,
            &search_key,
            &Wrap(data)
        )
        .await
    }
}
