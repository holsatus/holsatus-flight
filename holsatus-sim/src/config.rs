use std::io::Read;

use serde::{Deserialize, Serialize};

use crate::{
    distortion::Distortion,
    physics_sim::{Initial, MotorParams, VehicleParams},
    Configuration,
};

pub fn load_from_file_path(path: &str) -> Result<Configuration, Box<dyn std::error::Error>> {
    let mut file = std::fs::File::open(path)?;
    let mut string = String::with_capacity(1024);
    file.read_to_string(&mut string)?;
    let config: ToplevelConfig = toml::from_str(&string)?;
    Ok(config.into())
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct ToplevelConfig {
    #[serde(default)]
    simulation: SimConfig,
    #[serde(default)]
    initial: InitialConfig,
    vehicle: VehicleConfig,
    #[serde(default)]
    acc_dist: DistortConfig,
    #[serde(default)]
    gyr_dist: DistortConfig,
}

impl Into<Configuration> for ToplevelConfig {
    fn into(self) -> Configuration {
        Configuration {
            vehicle: VehicleParams {
                mass: self.vehicle.mass,
                local_com: self.vehicle.center_of_mass.into(),
                principal_inertia: self.vehicle.principal_inertia.into(),
                lin_damp: self.vehicle.linear_damp,
                ang_damp: self.vehicle.angular_damp,
                motors: self
                    .vehicle
                    .motors
                    .iter()
                    .map(|motor| MotorParams {
                        mass_moment: motor.mass_moment.abs()
                            * if motor.reverse { -1.0 } else { 1.0 },
                        position: motor.position.into(),
                        normal: motor.normal.into(),
                        time_constant: motor.time_constant,
                        motor_map: motor.motor_map,
                    })
                    .collect(),
            },
            initial: Initial {
                rotation: self.initial.rotation.into(),
                position: self.initial.position.into(),
                ang_velocity: self.initial.ang_velocity.into(),
                lin_velocity: self.initial.lin_velocity.into(),
            },
            accelerometer: Distortion::new_from_cfg(self.acc_dist).unwrap(),
            gyroscope: Distortion::new_from_cfg(self.gyr_dist).unwrap(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimConfig {
    simulation_rate: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            simulation_rate: 1000.0,
        }
    }
}

/// Configuration for  initial condition of the drone within a simulation.
///
/// Any fields not defined will be initialized with zero.
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct InitialConfig {
    #[serde(default)]
    pub rotation: [f32; 3],
    #[serde(default)]
    pub position: [f32; 3],
    #[serde(default)]
    pub ang_velocity: [f32; 3],
    #[serde(default)]
    pub lin_velocity: [f32; 3],
}

/// Configuration for single motor.
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct MotorConfig {
    name: String,
    reverse: bool,
    mass_moment: f32,
    position: [f32; 3],
    normal: [f32; 3],
    time_constant: f32,
    motor_map: (f32, f32),
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    /// The total mass of the vehicle
    pub mass: f32,
    /// The local center of mass of the vehicle
    #[serde(default)]
    pub center_of_mass: [f32; 3],
    /// The angular inertial along the principal axes
    pub principal_inertia: [f32; 3],
    /// Linear velocity damping (air resistance)
    #[serde(default)]
    pub linear_damp: f32,
    /// Angular velocity damping (air resistance)
    #[serde(default)]
    pub angular_damp: f32,
    /// Motors configuration
    #[serde(default)]
    pub motors: Vec<MotorConfig>,
}

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct DistortConfig {
    pub noise: Option<[f32; 3]>,
    pub bias: Option<[f32; 3]>,
    pub warp: Option<[[f32; 3]; 3]>,
    pub lowpass_freq: Option<f32>,
    pub max_range: Option<f32>,
}
