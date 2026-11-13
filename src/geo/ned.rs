use crate::geo::{Ecef, Lla};

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
}
