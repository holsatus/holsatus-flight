mod ublox;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GnssFixType {
    NoFix = 0 & 1,
    Fix2D = 2,
    Fix3D = 3,
    Rtcm = 4,
    RtkFloat = 5,
    RtkFixed = 6,
    Extrapolated = 8,
}

impl Default for GnssFixType {
    fn default() -> Self {
        GnssFixType::NoFix
    }
}

impl GnssFixType {
    pub fn has_fix(&self) -> bool {
        GnssFixType::Fix2D as u8 >= 2
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct GnssPacket {
    pub timestamp: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub altitude: f32,
    pub altitude_ellipsoid_m: f32, // 	# Altitude above Ellipsoid, meters

    pub s_variance_m_s: f32, //# GPS speed accuracy estimate, (metres/sec)
    pub c_variance_rad: f32, //# GPS course accuracy estimate, (radians)
    pub fix: GnssFixType,    //

    pub eph: f32, // GPS horizontal position accuracy (metres)
    pub epv: f32, // GPS vertical position accuracy (metres)

    pub hdop: f32, // Horizontal dilution of precision
    pub vdop: f32, // Vertical dilution of precision

    pub satellites: u8, // Number of satellites used

    pub vel_m_s: f32,   // GPS ground speed, (metres/sec)
    pub vel_n_m_s: f32, // GPS North velocity, (metres/sec)
    pub vel_e_m_s: f32, // GPS East velocity, (metres/sec)
    pub vel_d_m_s: f32, // GPS Down velocity, (metres/sec)
    pub cog_rad: f32,   // Course over ground, -PI..PI, (radians)
}

impl Into<crate::geo::Waypoint> for GnssPacket {
    fn into(self) -> crate::geo::Waypoint {
        crate::geo::Waypoint {
            lat: self.lat_deg,
            lon: self.lon_deg,
        }
    }
}
