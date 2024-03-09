use defmt::*;
use embassy_rp::{
    i2c::{Async, Error, I2c},
    peripherals::I2C0,
};
use embassy_time::Delay;
use icm20948_async::{I2cAddress, Icm20948, Icm20948Config, IcmBusI2c, Init, MagEnabled};

impl Into<super::types::ImuData6Dof> for icm20948_async::Data6Dof {
    fn into(self) -> super::types::ImuData6Dof {
        super::types::ImuData6Dof {
            acc: self.acc,
            gyr: self.gyr,
        }
    }
}

impl Into<super::types::ImuData9Dof> for icm20948_async::Data9Dof {
    fn into(self) -> super::types::ImuData9Dof {
        super::types::ImuData9Dof {
            acc: self.acc,
            gyr: self.gyr,
            mag: self.mag,
        }
    }
}

impl Into<super::types::ImuData6Dof> for icm20948_async::Data9Dof {
    fn into(self) -> super::types::ImuData6Dof {
        super::types::ImuData6Dof {
            acc: self.acc,
            gyr: self.gyr,
        }
    }
}

pub async fn setup_icm20948(
    async_i2c_imu: I2c<'_, I2C0, Async>,
    icm_conf: Icm20948Config,
) -> Option<Icm20948<IcmBusI2c<I2c<'_, I2C0, Async>>, MagEnabled, Init, Delay, Error>> {
    let imu_result = Icm20948::new_i2c_from_cfg(async_i2c_imu, icm_conf, Delay)
        .set_address(I2cAddress::X69)
        .initialize_9dof()
        .await;
    Some(match imu_result {
        Ok(imu) => imu,
        Err(error) => {
            match error {
                icm20948_async::IcmError::BusError(_) => error!("[ICM]: Communication bus error"),
                icm20948_async::IcmError::ImuSetupError => error!("[ICM]: Error during setup"),
                icm20948_async::IcmError::MagSetupError => error!("[ICM]: Error during mag setup"),
            }
            return None;
        }
    })
}
