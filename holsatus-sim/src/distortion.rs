use common::filters::Lowpass;
use nalgebra::{Matrix3, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

use crate::config::DistortConfig;


type E = Box<dyn std::error::Error>;

#[derive(Debug, Clone)]
pub struct Distortion {
    noise: Option<[Normal<f32>; 3]>,
    bias: Option<Vector3<f32>>,
    warp: Option<Matrix3<f32>>,
    dlpf: Option<[Lowpass<f32>; 3]>,
    clip: Option<f32>,
    rng: StdRng,
}

impl Default for Distortion {
    fn default() -> Self {
        Self {
            noise: None,
            bias: None,
            warp: None,
            dlpf: None,
            clip: None,
            rng: StdRng::seed_from_u64(0),
        }
    }
}

impl Distortion {
    pub fn new_from_cfg(cfg: DistortConfig) -> Result<Self, E> {
        Self::new_from_cfg_with_default_seed(cfg, 0)
    }

    /// Same as `new_from_cfg` but uses `default_seed` when the config does
    /// not specify one. The caller passes a distinct default per channel
    /// (e.g. acc vs gyr) so noise streams are uncorrelated by default.
    pub fn new_from_cfg_with_default_seed(cfg: DistortConfig, default_seed: u64) -> Result<Self, E> {

        let noise = if let Some(noise) = cfg.noise {
            Some([
                Normal::new(0.0, noise[0])?,
                Normal::new(0.0, noise[1])?,
                Normal::new(0.0, noise[2])?,
            ])
        } else {
            None
        };

        let bias = cfg.bias.map(|bias|Vector3::from(bias));
        let warp = cfg.warp.map(|warp|{
            let rows = warp.map(|row|Vector3::from(row).transpose());
            Matrix3::from_rows(&rows)
        });

        let dlpf = cfg.lowpass_freq.map(|dlpf| {
            let tau = 1.0 / dlpf;
            [
                Lowpass::new(tau, 0.001),
                Lowpass::new(tau, 0.001),
                Lowpass::new(tau, 0.001),
            ]
        });

        let clip = cfg.max_range;
        let rng = StdRng::seed_from_u64(cfg.seed.unwrap_or(default_seed));

        Ok(Self {
            noise,
            bias,
            warp,
            dlpf,
            clip,
            rng,
        })
    }

    pub fn apply(&mut self, input: impl Into<Vector3<f32>>) -> Vector3<f32> {
        let mut output = input.into();

        if let Some(noise) = self.noise {
            output = output + Vector3::new(
                noise[0].sample(&mut self.rng),
                noise[1].sample(&mut self.rng),
                noise[2].sample(&mut self.rng)
            );
        }

        if let Some(bias) = self.bias {
            output = bias + output;
        }

        if let Some(warp) = self.warp {
            output = warp * output;
        }

        if let Some(dlpf) = self.dlpf.as_mut() {
            output = Vector3::new(
                dlpf[0].update(output.x),
                dlpf[1].update(output.y),
                dlpf[2].update(output.z),
            );
        }

        // Intended for 16-bit imu measurements, should perhaps be its own filter (quantizer+clipping)
        if let Some(clip) = self.clip {
            output = output.map(|v| {
                let scalar = clip / i16::MAX as f32;
                (v * scalar) / scalar
            });
        }

        output
    }
}
