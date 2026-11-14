use crate::geo::{GeoError, WGS84_A, WGS84_E2};

/// Geodetic coordinates (latitude, longitude, ellipsoidal height).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lla {
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub alt_m: f64,
}

/// Earth-Centered, Earth-Fixed Cartesian coordinates in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ecef {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Ecef {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl Lla {
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Result<Self, GeoError> {
        if !lat_rad.is_finite() || !lon_rad.is_finite() || !alt_m.is_finite() {
            return Err(GeoError::NonFinite);
        }
        let half_pi = core::f64::consts::FRAC_PI_2;
        if lat_rad < -half_pi || lat_rad > half_pi {
            return Err(GeoError::InvalidLatitude);
        }
        let pi = core::f64::consts::PI;
        if lon_rad < -pi || lon_rad > pi {
            return Err(GeoError::InvalidLongitude);
        }
        Ok(Self {
            lat_rad,
            lon_rad,
            alt_m,
        })
    }

    pub fn from_degrees(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Result<Self, GeoError> {
        Self::new(lat_deg.to_radians(), lon_deg.to_radians(), alt_m)
    }
}
