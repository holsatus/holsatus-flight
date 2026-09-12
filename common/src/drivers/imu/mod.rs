pub mod bmi088;
pub mod bmi270;
pub mod icm20948;

pub use bmi088::{Bmi088Config, Bmi088I2c, Bmi088Spi};
pub use bmi270::{Bmi270Config, Bmi270I2c, Bmi270Spi};
pub use icm20948::{Icm209486DofI2c, Icm209486DofSpi};

/// Map a set of values given in degrees into radians (also works for angular velocities).
fn map_deg_to_rad(arr: [f32; 3]) -> [f32; 3] {
    arr.map(|v| v.to_radians())
}

/// Map a set of values given in Gs of gravity into meters/second^2.
fn map_g_to_mpss(arr: [f32; 3]) -> [f32; 3] {
    arr.map(|v| v * crate::consts::GRAVITY)
}
