use common::abstraction::dshot_group::DshotCommand;
use common::abstraction::dshot_group::DshotGroup;
use common::abstraction::dshot_group::THROTTLE_MAX;
use common::abstraction::dshot_group::THROTTLE_MIN;
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
        ImuIndex,
        abstraction::imu::{Imu, ImuInitialize},
        errors::SensorError,
        tasks::imu_reader::ImuReader,
        types::measurements::Imu6DofData,
    };
    use embassy_time::{Duration, Instant, Ticker};
    use holsatus_sim::SimulatedImu;

    #[embassy_executor::task]
    pub async fn main(imu: SimulatedImu) {
        struct SimImu<'a>(&'a mut SimulatedImu);

        impl ImuInitialize for SimImu<'_> {
            type Config = ();
            type Interface = SimulatedImu;
            type Sensor<'a>
                = SimImu<'a>
            where
                Self: 'a;

            async fn initialize<'a>(
                interface: &'a mut Self::Interface,
                _config: &Self::Config,
            ) -> Result<Self::Sensor<'a>, SensorError>
            where
                Self: 'a,
            {
                Ok(SimImu(interface))
            }
        }

        impl Imu for SimImu<'_> {
            fn read_acc(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>> {
                async { Ok(self.0.read_sim_acc()) }
            }
            fn read_gyr(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>> {
                async { Ok(self.0.read_sim_gyr()) }
            }
            fn read_acc_gyr(
                &mut self,
            ) -> impl Future<Output = Result<Imu6DofData<f32>, SensorError>> {
                async {
                    Ok(Imu6DofData {
                        timestamp_us: Instant::now().as_micros(),
                        gyr: self.0.read_sim_gyr(),
                        acc: self.0.read_sim_acc(),
                    })
                }
            }
        }

        let trigger = Ticker::every(Duration::from_hz(crate::SIM_FREQUENCY));
        ImuReader::entry::<SimImu<'_>>(ImuIndex::Imu0, imu, (), trigger).await
    }
}

#[embassy_executor::task]
pub async fn motor_governor(motors: SimulatedMotors) {
    struct Motors(SimulatedMotors);

    impl DshotGroup for Motors {
        async fn send_packets(
            &mut self,
            packets: [common::abstraction::dshot_group::DshotPacket; 4],
        ) {
            // Here we assume that if at least one packet is a throttle command,
            // then all are. It is a bit crude but it works.
            let speeds = packets.map(|packet| packet.as_speed().unwrap_or_default());
            if speeds.iter().any(|speed| *speed >= THROTTLE_MIN) {
                let speeds = speeds.map(|speed| speed.clamp(THROTTLE_MIN, THROTTLE_MAX));
                self.0.set_motor_speeds(speeds);
                return;
            }

            // From this point we ensure the motors should not be spinning
            self.0.set_motor_speeds_min();

            // Currently we do not even support reversing motors
            let mut reverse = [false; 4];
            for (index, packet) in packets.iter().enumerate() {
                if packet.get_raw() == DshotCommand::SpinDirectionReversed as u16 {
                    reverse[index] = true
                }
            }
            self.0.set_reverse_dir(reverse);
        }
    }

    let motors = Motors(motors);

    common::actuators::motor_governor::main(motors).await
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
    common::params::entry(flash, range).await
}
