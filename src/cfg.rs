use embassy_time::Duration;
use nalgebra::Matrix3;

pub static SBUS_PARSE_TIMEOUT: Duration = Duration::from_millis(500);
pub static MOTOR_GOV_TIMEOUT: Duration = Duration::from_millis(100);
pub static ATTITUDE_LOOP_TIME_DUR: Duration = Duration::from_hz(400);
pub static ATTITUDE_LOOP_TIME_SECS: f32 = ATTITUDE_LOOP_TIME_DUR.as_micros() as f32 / 1e6;

pub static TAKEOFF_ESTIMATOR_THRUST: u16 = 700;

pub static PIO_DSHOT_SPEED: DshotSpeed = DshotSpeed::Dshot300;

pub static IMU_ROTATION: Matrix3<f32> = Matrix3::new(
    1., 0., 0.,
    0., -1., 0.,
    0., 0., -1.,
);

#[cfg(feature = "mag")]
pub static MAG_ROTATION: Matrix3<f32> = Matrix3::new(
    1., 0., 0.,
    0., 1., 0.,
    0., 0., 1.,
);

#[allow(unused)]
pub enum DshotSpeed {
    Dshot150,
    Dshot300,
    Dshot600,
    Dshot1200,
}

impl DshotSpeed {
    pub fn clk_div(&self) -> (u16, u8) {
        match self {
            DshotSpeed::Dshot150    => (104, 0),
            DshotSpeed::Dshot300    => (52, 0),
            DshotSpeed::Dshot600    => (26, 0),
            DshotSpeed::Dshot1200   => (13, 0),
        }
    }
}