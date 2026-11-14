//! Vincenty inverse geodetic solution on WGS84.

/// WGS84 semi-major axis in metres.
pub const A: f64 = 6_378_137.0;
/// WGS84 flattening.
pub const F: f64 = 1.0 / 298.257_223_563;
const B: f64 = A * (1.0 - F);

/// Result of the inverse problem.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inverse {
    /// Ellipsoidal distance in metres.
    pub distance_m: f64,
    /// Forward azimuth at the first point in radians, from north.
    pub azimuth1: f64,
    /// Forward azimuth at the second point in radians, from north.
    pub azimuth2: f64,
}
