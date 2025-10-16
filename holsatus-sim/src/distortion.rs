use common::filters::Lowpass;
use nalgebra::{Matrix3, Vector3};
use rand_distr::{Distribution, Normal};

use crate::config::DistortConfig;


type E = Box<dyn std::error::Error>;

#[derive(Debug, Clone, Default)]
pub struct Distortion {
    noise: Option<[Normal<f32>; 3]>,
    bias: Option<Vector3<f32>>,
    warp: Option<Matrix3<f32>>,
    dlpf: Option<[Lowpass<f32>; 3]>,
    clip: Option<f32>,
}

impl Distortion {
    pub fn new_from_cfg(cfg: DistortConfig) -> Result<Self, E> {

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

        Ok(Self {
            noise,
            bias,
            warp,
            dlpf,
            clip,
        })
    }

    pub fn apply(&mut self, input: impl Into<Vector3<f32>>) -> Vector3<f32> {
        let mut output = input.into();

        if let Some(noise) = self.noise {
            let mut rng = rand::rng();
            output = output + Vector3::new(
                noise[0].sample(&mut rng),
                noise[1].sample(&mut rng),
                noise[2].sample(&mut rng)
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
