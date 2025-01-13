pub mod acc_routine;
pub mod gyr_routine;
pub mod mag_routine;
pub mod sens3d;

#[derive(Debug, Clone)]
pub enum Calibrate {
    Acc((AccCalib, Option<u8>)),
    Gyr((GyrCalib, Option<u8>)),
    Mag((MagCalib, Option<u8>)),
}

#[derive(Debug, Clone, Default)]
pub struct AccCalib {
    pub max_var: f32,
    pub max_dropped: usize,
}

#[derive(Debug, Clone)]
pub struct GyrCalib {
    pub max_var: f32,
    pub duration_s: u8,
    pub max_dropped: usize,
}

impl Default for GyrCalib {
    fn default() -> Self {
        Self {
            max_var: 0.01,
            duration_s: 5,
            max_dropped: 10,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MagCalib {
    pub duration_s: u8,
    pub pre_scalar: f32,
    pub std_limit: f32,
    pub max_dropped: usize,
}
