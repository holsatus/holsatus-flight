use crate::errors::SensorError;

pub trait Magnetometer {
    fn read_mag(&mut self) -> impl Future<Output = Result<[f32; 3], SensorError>>;
}
