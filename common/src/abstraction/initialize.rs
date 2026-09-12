use crate::errors::SensorError;

/// A sensor which knows how to initialize itself and use a specific interface kind.
pub trait Initialize {
    type Config;
    type Interface;
    type Sensor<'a>: Sized
    where
        Self: 'a;

    /// Try to initialize the sensor using a borrowed interface and configuration.
    fn initialize<'a>(
        interface: &'a mut Self::Interface,
        config: &Self::Config,
    ) -> impl Future<Output = Result<Self::Sensor<'a>, SensorError>>
    where
        Self: 'a;
}
