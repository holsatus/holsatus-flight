use nalgebra::Vector3;

#[derive(Clone, Copy, Debug)]
pub struct ImuData {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Option<Vector3<f32>>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ImuData6Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
}

#[derive(Clone, Copy, Debug)]
pub struct ImuData9Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Vector3<f32>,
}

impl Into<ImuData6Dof> for ImuData {
    fn into(self) -> ImuData6Dof {
        ImuData6Dof {
            acc: self.acc,
            gyr: self.gyr,
        }
    }
}

impl Into<ImuData> for ImuData6Dof {
    fn into(self) -> ImuData {
        ImuData {
            acc: self.acc,
            gyr: self.gyr,
            mag: None,
        }
    }
}

pub enum ImuVariant {
    Icm20948,
}
