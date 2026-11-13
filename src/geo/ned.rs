use crate::geo::{lla_to_ecef, Ecef, Lla};

/// Local Tangent Plane North-East-Down (NED) coordinates in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ned {
    pub north: f64,
    pub east: f64,
    pub down: f64,
}

impl Ned {
    pub const fn new(north: f64, east: f64, down: f64) -> Self {
        Self { north, east, down }
    }
}

/// Local tangent plane origin defined by an LLA and its ECEF position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LtpFrame {
    pub origin_lla: Lla,
    pub origin_ecef: Ecef,
    sin_lat: f64,
    cos_lat: f64,
    sin_lon: f64,
    cos_lon: f64,
}

impl LtpFrame {
    pub fn from_lla(origin: &Lla) -> Self {
        let origin_ecef = lla_to_ecef(origin);
        let sin_lat = libm::sin(origin.lat_rad);
        let cos_lat = libm::cos(origin.lat_rad);
        let sin_lon = libm::sin(origin.lon_rad);
        let cos_lon = libm::cos(origin.lon_rad);
        Self {
            origin_lla: *origin,
            origin_ecef,
            sin_lat,
            cos_lat,
            sin_lon,
            cos_lon,
        }
    }
}
