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

pub fn lla_to_ecef(_lla: &Lla) -> Ecef {
    Ecef::new(0.0, 0.0, 0.0)
}
