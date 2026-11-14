//! Haversine great-circle distance and initial bearing.

use super::GeoError;

/// WGS-84 mean Earth radius in meters.
pub const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Haversine great-circle distance and initial bearing solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HaversineResult {
    /// Great-circle distance along a spherical Earth in meters.
    pub distance_m: f64,
    /// Initial forward bearing in radians from true north, in `[0, 2*pi)`.
    pub bearing_rad: f64,
}
