use common::hw_abstraction::OutputGroup;
use common::nalgebra::SMatrix;
use common::nalgebra::SVector;
use common::types::measurements::ViconData;
use embassy_time::Duration;
use embassy_time::Ticker;
use holsatus_sim::Sim as _;
use holsatus_sim::SimHandle;
use holsatus_sim::SimulatedFlash;
use holsatus_sim::SimulatedMotors;
use rand_distr::Distribution as _;
use rand_distr::Normal;

pub mod imu_reader {
    use common::{
        drivers::imu::{ImuInitialize, ImuSensor},
        errors::ImuError,
        types::measurements::Imu6DofData,
    };
    use embassy_time::{Duration, Instant, Ticker};
    use holsatus_sim::SimulatedImu;

    #[embassy_executor::task]
    pub async fn main(imu: SimulatedImu) {
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
                    Ok(Imu6DofData {
                        timestamp_us: Instant::now().as_micros(),
                        gyr: self.0.read_sim_gyr(),
                        acc: self.0.read_sim_acc(),
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

#[embassy_executor::task]
pub async fn simulated_vicon(handle: SimHandle) {
    // 10 Hz, similar to a GPS
    let mut interval = Ticker::every(Duration::from_millis(10));

    loop {
        let vicon_data = {
            let mut rng = rand::rng();
            let noise_std = 0.005f32;
            let distr = Normal::new(0.0, noise_std).unwrap();

            let state = handle.vehicle_state();
            let angles = state.rotation.euler_angles();
            let vicon_data = ViconData {
                timestamp_us: handle.timestamp_us(),
                position: (state.position + SVector::from_fn(|_, _| distr.sample(&mut rng))).into(),
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
        interval.next().await;
    }
}

#[embassy_executor::task]
pub async fn param_storage(flash: SimulatedFlash) {
    let range = flash.range_u32();
    common::tasks::param_storage::entry(flash, range).await
}
