#[cfg(feature = "rerun")]
pub fn setup_logging(
    mock: SimHandle,
    test_name: &str,
) -> Result<crate::logger::RerunLogger, Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new(test_name).spawn()?;

    rerun::Logger::new(rec.clone())
        .with_filter("off, common=debug, holsatus_sim=debug, std_device=debug")
        .init()?;

    crate::logger::RerunLogger::new(rec, mock)
}

use common::hw_abstraction::Imu6Dof;
use common::hw_abstraction::OutputGroup;
use common::types::measurements::Imu6DofData;
use common::types::measurements::ViconData;
use embassy_time::Duration;
use embassy_time::Ticker;
use holsatus_sim::Sim as _;
use holsatus_sim::SimHandle;
use holsatus_sim::SimulatedFlash;
use holsatus_sim::SimulatedImu;
use holsatus_sim::SimulatedMotors;
use rand_distr::Distribution as _;
use rand_distr::Normal;
use rapier3d::na::SMatrix;
use rapier3d::na::SVector;

#[embassy_executor::task]
pub async fn imu_reader(imu: SimulatedImu) {
    struct Imu(SimulatedImu);

    impl Imu6Dof for Imu {
        async fn read_acc(&mut self) -> Result<[f32; 3], common::errors::DeviceError> {
            Ok(self.0.read_acc())
        }

        async fn read_gyr(&mut self) -> Result<[f32; 3], common::errors::DeviceError> {
            Ok(self.0.read_gyr())
        }

        async fn read_acc_gyr(
            &mut self,
        ) -> Result<common::types::measurements::Imu6DofData<f32>, common::errors::DeviceError>
        {
            let (acc, gyr) = self.0.read_acc_gyr();
            Ok(Imu6DofData {
                timestamp_us: self.0.sim.timestamp_us(),
                gyr,
                acc,
            })
        }
    }

    let imu = Imu(imu);

    common::tasks::imu_reader::main_6dof(imu).await
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
