use crate::{errors::SensorError, types::measurements::BarometerData};

pub trait Barometer {
    fn read_data(&mut self) -> impl Future<Output = Result<BarometerData, SensorError>>;
}
