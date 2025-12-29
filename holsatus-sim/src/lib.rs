use std::f32::consts::PI;
use std::ops::Range;
use std::sync::{Arc, RwLock};

mod distortion;
mod motor_lin;
pub mod physics_sim;
pub mod config;

use common::utils::func::wrap;
use distortion::{Distortion};
use nalgebra::SMatrix;
use physics_sim::{Simulation, VehicleParams};

#[derive(Debug, Clone)]
pub struct Configuration {
    pub vehicle: VehicleParams,
    pub initial: Initial,
    pub accelerometer: Distortion,
    pub gyroscope: Distortion,
}

#[derive(Clone)]
pub struct SimHandle(Arc<RwLock<Simulation>>);

pub trait Sim {
    fn vehicle_state(&self) -> VehicleState;
    fn set_motors_cmd(&self, cmd: &[f32]);
    fn set_motors_dir(&self, dir: &[bool]);
    fn make_upright(&self);
    fn reset_position(&self);
    fn step(&self, ts: f32);
}

impl SimHandle {
    pub fn new(config: Configuration) -> Result<Self, Box<dyn std::error::Error>> {
        let mock = Simulation::new(config.vehicle, config.initial);

        Ok(Self(Arc::new(RwLock::new(mock))))
    }

    pub fn timestamp_us(&self) -> u64 {
        self.0.read().unwrap().start_time.elapsed().as_micros() as u64
    }

    pub fn vehicle_params(&self) -> VehicleParams {
        self.0.read().unwrap().params.clone()
    }
}

impl Sim for SimHandle {
    fn vehicle_state(&self) -> VehicleState {
        self.0.read().unwrap().state.clone()
    }

    fn set_motors_cmd(&self, cmd: &[f32]) {
        let mut this = self.0.write().unwrap();
        this.set_motors_cmd(cmd);
    }

    fn set_motors_dir(&self, dir: &[bool]) {
        let mut this = self.0.write().unwrap();
        this.set_motors_dir(dir);
    }

    fn make_upright(&self) {
        let mut this = self.0.write().unwrap();
        this.make_upright();
    }

    fn reset_position(&self) {
        let mut this = self.0.write().unwrap();
        this.reset_position();
    }

    fn step(&self, dt: f32) {
        let mut this = self.0.write().unwrap();
        this.update(dt);
    }
}

pub fn initialize(config: Configuration) -> Result<(Resources, SimHandle), Box<dyn std::error::Error>> {

    let vehicle = SimHandle::new(config.clone())?;

    let imu = SimulatedImu::new(vehicle.clone(), &config);
    let motors = SimulatedMotors::new(vehicle.clone());
    let vicon = SimulatedVicon::new(vehicle.clone());
    let flash = SimulatedFlash::new(1024 * 1024);

    Ok((Resources { imu, motors, vicon, flash }, vehicle))
}

pub struct Resources {
    pub imu: SimulatedImu,
    pub motors: SimulatedMotors,
    pub vicon: SimulatedVicon,
    pub flash: SimulatedFlash,
}

use common::nalgebra::Vector3;
use embedded_storage_async::nor_flash::ErrorType;
use embedded_storage_async::nor_flash::NorFlash;
use embedded_storage_async::nor_flash::ReadNorFlash;
use rand_distr::Distribution;
use rand_distr::Normal;

use crate::physics_sim::{Initial, VehicleState};

pub struct SimulatedImu {
    pub sim: SimHandle,
    acc_dist: Distortion,
    gyr_dist: Distortion,
}

impl SimulatedImu {
    pub fn new(mock: SimHandle, config: &Configuration) -> Self {
        Self {
            sim: mock,
            acc_dist: config.accelerometer.clone(),
            gyr_dist: config.gyroscope.clone(),
        }
    }

    pub fn acc_distortion(mut self, acc: Distortion) -> Self {
        self.acc_dist = acc;
        self
    }

    pub fn gyr_distortion(mut self, gyr: Distortion) -> Self {
        self.gyr_dist = gyr;
        self
    }
}

impl SimulatedImu {
    pub fn read_acc(&mut self) -> [f32; 3] {
        let state = self.sim.vehicle_state();
        self.acc_dist.apply(state.body_acc).into()
    }

    pub fn read_gyr(&mut self) -> [f32; 3] {
        let state = self.sim.vehicle_state();
        self.gyr_dist.apply(state.body_gyr).into()
    }

    pub fn read_acc_gyr(&mut self) -> ([f32; 3], [f32; 3]) {
        (self.read_acc(), self.read_gyr())
    }
}

pub struct SimulatedMotors {
    sim: SimHandle,
}

impl SimulatedMotors {
    pub fn new(mock: SimHandle) -> Self {
        SimulatedMotors { sim: mock }
    }
}

impl SimulatedMotors {
    pub fn set_motor_speeds(&mut self, speeds: [u16; 4]) {
        self.sim.set_motors_cmd(&speeds.map(|s|s.saturating_sub(47) as f32 / 2000.0));
    }

    pub fn set_motor_speeds_min(&mut self) {
        self.sim.set_motors_cmd(&[0.0; 4]);
    }

    pub fn set_reverse_dir(&mut self, rev: [bool; 4]) {
        self.sim.set_motors_dir(&rev);
    }

    pub fn make_beep(&mut self) {
        log::trace!("Beep not supported in simulator")
    }
}

// Hmm, maybe this should be handled via the MAVLink connection instead?

pub struct SimulatedVicon {
    sim: SimHandle,
    pos_noise: Normal<f32>,
    att_noise: Normal<f32>,
}

impl SimulatedVicon {
    pub fn new(mock: SimHandle) -> Self {
        Self {
            sim: mock,
            pos_noise: Normal::new(0.0, 0.1).unwrap(),
            att_noise: Normal::new(0.0, 0.1).unwrap(),
        }
    }

    pub async fn read_packet(&mut self) -> common::types::measurements::ViconData {
        let mut rng = rand::rng();
        let position = self.sim.vehicle_state().position;
        let attitude = self.sim.vehicle_state().rotation;
        let (roll, pitch, yaw) = attitude.euler_angles();

        let position = Vector3::new(
            self.pos_noise.sample(&mut rng),
            self.pos_noise.sample(&mut rng),
            self.pos_noise.sample(&mut rng),
        ) + Vector3::new(position[0], position[1], position[2]);

        let attitude = Vector3::new(
            self.att_noise.sample(&mut rng),
            self.att_noise.sample(&mut rng),
            self.att_noise.sample(&mut rng),
        ) + Vector3::new(roll, pitch, yaw);

        // Ensure noise does not do weird things to angle
        let attitude = attitude.map(|angle|wrap(angle, -PI, PI));

        common::types::measurements::ViconData {
            timestamp_us: self.sim.timestamp_us(),
            position: position.into(),
            pos_var: SMatrix::from_diagonal_element(self.pos_noise.std_dev().powi(2)).data.0,
            attitude: attitude.into(),
            att_var: SMatrix::from_diagonal_element(self.pos_noise.std_dev().powi(2)).data.0,
        }
    }
}

pub struct SimulatedFlash {
    buf: Vec<u8>,
}

impl SimulatedFlash {
    pub fn new(cap: usize) -> Self {
        Self {
            // Flash is erased to logical high
            buf: vec![u8::MAX; cap],
        }
    }

    pub fn range_u32(&self) -> Range<u32> {
        0..self.buf.capacity() as u32
    } 
}

impl ErrorType for SimulatedFlash {
    type Error = core::convert::Infallible;
}

impl ReadNorFlash for SimulatedFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        bytes.copy_from_slice(&self.buf[offset as usize..offset as usize + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.buf.capacity()
    }
}

impl NorFlash for SimulatedFlash {
    const WRITE_SIZE: usize = 1;

    // Somewhat realistic block size?
    // Though, we do not error for unaligned erases
    const ERASE_SIZE: usize = 512;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.buf[from as usize..to as usize].fill(u8::MAX);
        Ok(())
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.buf[offset as usize..offset as usize + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}