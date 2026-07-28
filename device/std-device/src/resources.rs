pub fn setup_logging(mock: SimHandle) -> Result<(), Box<dyn std::error::Error>> {
    // If nothing else is specified, we fall back to env_logger to log messages.
    #[cfg(not(feature = "rerun"))]
    {
        drop(mock);
        env_logger::builder()
            .filter_level(log::LevelFilter::Debug)
            .filter_module("async_io", log::LevelFilter::Info)
            .format_timestamp_nanos()
            .init();
    }

    // If rerun is enabled, we use that for logging
    #[cfg(feature = "rerun")]
    {
        let rec = rerun::RecordingStreamBuilder::new("holsatus_std_rerun").spawn()?;

        rerun::Logger::new(rec.clone())
            .with_filter("off, common=trace, holsatus-sim=debug, std-device=debug")
            .init()?;

        std::thread::spawn(|| {
            use crate::rerun_logger::rerun_thread;
            rerun_thread(rec, mock).expect("An error occured in the rerun tread")
        });
    }

    Ok(())
}

use common::drivers::imu::ImuInitialize;
use common::drivers::imu::ImuSensor;
use common::embassy_futures::select::Either;
use common::embassy_futures::select::select;
use common::embassy_time::Duration;
use common::embassy_time::Instant;
use common::embassy_time::Ticker;
use common::errors::ImuError;
use common::errors::adapter::embedded_io::EmbeddedIoError;
use common::grantable_io::GrantableIo;
use common::hw_abstraction::OutputGroup;
use common::serial::IoStreamRaw;
use common::types::measurements::Imu6DofData;
use common::types::measurements::ViconData;
use embedded_io::ErrorKind;
use holsatus_sim::Sim as _;
use holsatus_sim::SimHandle;
use holsatus_sim::SimulatedFlash;
use holsatus_sim::SimulatedImu;
use holsatus_sim::SimulatedMotors;
use rand_distr::Distribution;
use rand_distr::Normal;
use rapier3d::na::SMatrix;
use rapier3d::na::SVector;
use tokio::io::AsyncReadExt;
use tokio::io::AsyncWriteExt;

#[embassy_executor::task]
pub async fn imu_reader(imu: SimulatedImu) {
    struct Imu<'a>(&'a mut SimulatedImu);

    impl ImuInitialize for Imu<'_> {
        type Config = ();
        type Interface = SimulatedImu;
        type Sensor<'a>
            = Imu<'a>
        where
            Self: 'a;

        async fn initialize<'a>(
            interface: &'a mut Self::Interface,
            _config: &Self::Config,
        ) -> Result<Self::Sensor<'a>, ImuError>
        where
            Self: 'a,
        {
            Ok(Imu(interface))
        }
    }

    impl ImuSensor for Imu<'_> {
        fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
            async { Ok(self.0.read_sim_acc()) }
        }
        fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], ImuError>> {
            async { Ok(self.0.read_sim_gyr()) }
        }
        fn read_acc_gyr(&mut self) -> impl Future<Output = Result<Imu6DofData<f32>, ImuError>> {
            async {
                let (acc, gyr) = self.0.read_sim_acc_gyr();
                Ok(Imu6DofData {
                    timestamp_us: Instant::now().as_micros(),
                    gyr,
                    acc,
                })
            }
        }
    }

    common::tasks::imu_reader::ImuReader::entry::<Imu<'_>>(
        imu,
        (),
        Ticker::every(Duration::from_hz(1000)),
    )
    .await
}

#[embassy_executor::task]
pub async fn motor_governor(motors: SimulatedMotors) {
    struct Motors(SimulatedMotors);

    impl OutputGroup for Motors {
        async fn set_motor_speeds(&mut self, speeds: [u16; 4]) {
            self.0.set_motor_speeds(speeds)
        }
        async fn set_motor_speeds_min(&mut self) {
            self.0.set_motor_speeds_min();
        }
        async fn set_reverse_dir(&mut self, rev: [bool; 4]) {
            self.0.set_reverse_dir(rev);
        }
        async fn make_beep(&mut self) {
            self.0.make_beep()
        }
    }

    let motors = Motors(motors);

    common::tasks::motor_governor::main(motors).await
}

// // TODO Simulated vicon should be sent over MAVLink
// #[embassy_executor::task]
// pub async fn gnss_reader(mut gnss: SimulatedGnss) {
//     let mut ticker = Ticker::every(Duration::from_hz(100));
//     let snd_gnss_data = common::signals::RAW_GNSS_DATA.sender();

//     loop {
//         ticker.next().await;

//         if let Ok(packet) = gnss.read_packet().await {
//             log::trace!("Gnss Packet: {packet:?}");
//             snd_gnss_data.send(packet)
//         }
//     }
// }

pub fn run_simulated_vicon(handle: SimHandle) {
    crate::thread_executor::RUNTIME.spawn(async move {
        // 10 Hz, similar to a GPS
        let mut interval = tokio::time::interval(std::time::Duration::from_millis(10));

        loop {
            let vicon_data = {
                let mut rng = rand::rng();
                let noise_std = 0.005f32;
                let distr = Normal::new(0.0, noise_std).unwrap();

                let state = handle.vehicle_state();
                let angles = state.rotation.euler_angles();
                let vicon_data = ViconData {
                    timestamp_us: handle.timestamp_us(),
                    position: (state.position + SVector::from_fn(|_, _| distr.sample(&mut rng)))
                        .into(),
                    pos_var: (SMatrix::identity() * noise_std.powi(2)).data.0,
                    attitude: [
                        angles.0 + distr.sample(&mut rng),
                        angles.1 + distr.sample(&mut rng),
                        angles.2 + distr.sample(&mut rng),
                    ],
                    att_var: (SMatrix::identity() * noise_std.powi(2)).data.0,
                };

                vicon_data
            };

            common::signals::VICON_POSITION_ESTIMATE.send(vicon_data);
            interval.tick().await;
        }
    });
}

#[embassy_executor::task]
pub async fn param_storage(flash: SimulatedFlash) {
    let range = flash.range_u32();
    common::tasks::param_storage::entry(flash, range).await
}

pub(crate) fn simulation_runner(sitl: SimHandle, frequency: usize) {
    crate::thread_executor::RUNTIME.spawn(async move {
        let dt = 1.0 / frequency as f32;
        let mut interval = tokio::time::interval(std::time::Duration::from_secs_f32(dt));
        loop {
            sitl.step(dt);
            interval.tick().await;
        }
    });
}

pub(crate) fn new_tcp_serial_io(addr: &str, stream_id: &'static str) {
    fn make_static<T>(data: T) -> &'static mut T {
        Box::leak(Box::new(data))
    }

    let tx_grantable = make_static(GrantableIo::<256, EmbeddedIoError>::new());
    let rx_grantable = make_static(GrantableIo::<256, EmbeddedIoError>::new());

    let (mut device_tx, reader) = rx_grantable.claim_reader();
    let (mut device_rx, writer) = tx_grantable.claim_writer();

    let io_stream_raw = IoStreamRaw::new(stream_id, reader, writer);

    common::serial::insert(make_static(io_stream_raw)).unwrap();

    let addr = String::from(addr);

    crate::thread_executor::RUNTIME.spawn(async move {
        let Either::First(mut stream) = select(
            async {
                tokio::net::TcpListener::bind(addr)
                    .await
                    .expect("Failed to bind to address")
                    .accept()
                    .await
                    .expect("Failed to accept connection")
                    .0
            },
            async {
                loop {
                    // Drain the buffer and return an error to
                    // prevent the writer from waiting forever.
                    _ = device_rx.read(&mut [0u8; 32]).await;
                    device_rx.insert_error(EmbeddedIoError::NotConnected);
                }
            },
        )
        .await;

        let mut stream_buf = [0u8; 128];
        let mut device_buf = [0u8; 128];

        loop {
            // This can be quite fucky to keep track of. Perhaps supporting more .connect methods would be good
            match select(
                stream.read(stream_buf.as_mut()),
                device_rx.read(device_buf.as_mut()),
            )
            .await
            {
                Either::First(stream_res) => match stream_res {
                    Ok(bytes) => device_tx.write_all(&stream_buf[..bytes]).await,
                    Err(error) => device_tx.insert_error(ErrorKind::from(error.kind()).into()),
                },
                Either::Second(bytes) => match stream.write_all(&device_buf[..bytes]).await {
                    Ok(()) => (),
                    Err(error) => device_rx.insert_error(ErrorKind::from(error.kind()).into()),
                },
            }
        }
    });
}
