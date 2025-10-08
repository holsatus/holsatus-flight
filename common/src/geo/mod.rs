// Derived from PX4 geo.cpp
// https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/geo/geo.cpp

/// A waypoint describes a
#[derive(Clone, Copy)]
pub struct Waypoint {
    pub lat: f64,
    pub lon: f64,
}

pub struct Waypoint3D {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
}

impl Into<Waypoint> for Waypoint3D {
    fn into(self) -> Waypoint {
        Waypoint {
            lat: self.lat,
            lon: self.lon,
        }
    }
}

use core::f32::consts::PI;

use nalgebra::wrap;
#[allow(unused_imports)]
use num_traits::{Float, Zero};

use crate::consts::EARTH_RADIUS_F64;

impl Waypoint {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    pub fn distance_to_waypoint(&self, wayp: impl Into<Waypoint>) -> f32 {
        let wayp: Waypoint = wayp.into();

        let self_lat_rad = self.lat.to_radians();
        let wayp_lat_rad = wayp.lat.to_radians();

        let d_lat_half = (wayp_lat_rad - self_lat_rad) / 2.0;
        let d_lon_half = (wayp.lon.to_radians() - self.lon.to_radians()) / 2.0;

        let a = d_lat_half.sin() * d_lat_half.sin()
            + d_lon_half.sin() * d_lon_half.sin() * self_lat_rad.cos() * wayp_lat_rad.cos();

        let c = a.sqrt().atan2((1.0 - a).sqrt());

        (EARTH_RADIUS_F64 * 2.0 * c) as f32
    }

    /// Calculate the waypoint at a distance from the current waypoint, towards
    /// the target waypoint. This use useful for doing "carrot on a stick"
    /// navigation.
    pub fn waypoint_at_dist(&self, wayp: impl Into<Waypoint>, dist: f32) -> Waypoint {
        if dist.abs().is_zero() {
            return *self;
        }

        let wayp: Waypoint = wayp.into();

        let (bearing, _) = self.bearing_and_distance_to_waypoint(wayp);
        self.waypoint_from_bearing_and_distance(bearing, dist)
    }

    fn waypoint_from_bearing_and_distance(&self, bearing: f32, dist: f32) -> Waypoint {
        let bearing = wrap(bearing, 0., 2.0 * PI);

        let radius_ratio = dist as f64 / EARTH_RADIUS_F64;

        let lat_start_rad = self.lat.to_radians();
        let lon_start_rad = self.lon.to_radians();

        let lat_target = (lat_start_rad.sin() * radius_ratio.cos()
            + lat_start_rad.cos() * radius_ratio.sin() * (bearing as f64).cos())
        .acos();
        let lon_target = lon_start_rad
            + ((bearing as f64).sin() * radius_ratio.sin() * lat_start_rad.cos())
                .atan2(radius_ratio.cos() - lat_start_rad.sin() * lat_target.sin());

        Waypoint {
            lat: lat_target.to_degrees(),
            lon: lon_target.to_degrees(),
        }
    }

    pub fn change_x_y_to_waypoint(&self, wayp: impl Into<Waypoint>) -> (f32, f32) {
        let wayp: Waypoint = wayp.into();

        let self_lat_rad = self.lat.to_radians();
        let wayp_lat_rad = wayp.lat.to_radians();

        let cos_wayp_lat = wayp_lat_rad.cos();
        let delta_lon = (wayp.lon - self.lon).to_radians();

        let y = (delta_lon.sin() * cos_wayp_lat) as f32;
        let x = (self_lat_rad.cos() * wayp_lat_rad.sin()
            - self_lat_rad.sin() * cos_wayp_lat * delta_lon.cos()) as f32;

        (x, y)
    }

    pub fn bearing_and_distance_to_waypoint(&self, wayp: impl Into<Waypoint>) -> (f32, f32) {
        let (x, y) = self.change_x_y_to_waypoint(wayp);
        (wrap(y.atan2(x), -PI, PI), (x * x + y * y).sqrt())
    }
}
