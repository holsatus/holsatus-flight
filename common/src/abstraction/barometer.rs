use crate::errors::SensorError;

pub trait Barometer {
    fn read_pressure(&mut self) -> impl Future<Output = Result<Pressure, SensorError>>;
    fn read_temperature(&mut self) -> impl Future<Output = Result<Temperature, SensorError>>;
    fn read_pressure_and_temperature(
        &mut self,
    ) -> impl Future<Output = Result<(Pressure, Temperature), SensorError>>;
}

