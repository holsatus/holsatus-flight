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
    #[serde(default)]
    vibration: VibrationConfig,
    #[serde(default)]
    battery: BatteryConfig,
}

/// Battery voltage sag model. Thrust scales with V^2. Disabled when any
/// of these values are 0 (the default).
#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct BatteryConfig {
    /// Nominal voltage (V). 0 disables sag.
    #[serde(default)]
    pub v_nominal: f32,
    /// Battery internal resistance (ohms).
    #[serde(default)]
    pub r_internal: f32,
    /// Current draw per Newton of total thrust (A/N). Typical small quad:
    /// 2.5-4 A/N at hover.
    #[serde(default)]
    pub amps_per_n: f32,
}

/// Prop-wash + motor-imbalance vibration model coupling into the accelerometer.
/// Real-world prop thrust creates broadband accel noise that corrupts Madgwick's
/// tilt estimate; this is what flipped the real H743v2 in D000297/306/308. Set
/// `acc_stddev_per_thrust` to zero to disable.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VibrationConfig {
    /// Accel noise stddev per Newton of total thrust. ~0.03 is a realistic
    /// value for a small quad. 0 disables the model.
    #[serde(default)]
    pub acc_stddev_per_thrust: f32,
    /// RNG seed for reproducible vibration noise.
    #[serde(default = "default_vib_seed")]
    pub seed: u64,
}

fn default_vib_seed() -> u64 { 0x7162_0001 }

impl Default for VibrationConfig {
    fn default() -> Self {
        Self {
            acc_stddev_per_thrust: 0.0,
            seed: default_vib_seed(),
        }
    }
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
                ge_alpha: self.vehicle.ground_effect_alpha,
                ge_tau: self.vehicle.ground_effect_tau,
                pipeline_latency_steps: self.vehicle.pipeline_latency_steps,
                battery_v_nom: self.battery.v_nominal,
                battery_r_internal: self.battery.r_internal,
                battery_amps_per_n: self.battery.amps_per_n,
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
                        tau_up: motor.tau_up,
                        tau_down: motor.tau_down,
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
            accelerometer: Distortion::new_from_cfg_with_default_seed(self.acc_dist, 0xacc0_0001).unwrap(),
            gyroscope: Distortion::new_from_cfg_with_default_seed(self.gyr_dist, 0x6b40_0001).unwrap(),
            acc_vib_stddev_per_thrust: self.vibration.acc_stddev_per_thrust,
            acc_vib_seed: self.vibration.seed,
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
    /// Asymmetric spin-up time constant. 0 falls back to `time_constant`.
    #[serde(default)]
    tau_up: f32,
    /// Asymmetric spin-down time constant. 0 falls back to `time_constant`.
    #[serde(default)]
    tau_down: f32,
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
    /// Ground-effect boost amplitude. The per-motor thrust is multiplied by
    /// `1 + alpha * exp(-h_motor / tau)`. 0 disables the effect. Typical
    /// range for small quads is 0.15-0.35.
    #[serde(default)]
    pub ground_effect_alpha: f32,
    /// Decay length (m) of the ground-effect boost. Physically of order the
    /// prop radius; beyond ~3*tau the effect is negligible. 0 disables.
    #[serde(default)]
    pub ground_effect_tau: f32,
    /// End-to-end sensor-to-actuator pipeline delay in simulation steps.
    /// Covers DShot frame transmission, ESC compute, and task-switching
    /// lag on top of the inherent 1-step sampled-data delay. Set to
    /// round(extra_latency_ms * sim_rate_hz / 1000). 0 disables.
    #[serde(default)]
    pub pipeline_latency_steps: usize,
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
    /// Seed for the noise RNG. `None` -> fixed default so runs are
    /// reproducible without needing a config change. Override per-channel
    /// (acc vs gyr) to avoid correlated noise.
    pub seed: Option<u64>,
}
