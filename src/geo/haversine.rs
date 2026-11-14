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

fn validate_coords(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> Result<(), GeoError> {
    if !lat1.is_finite() || !lon1.is_finite() || !lat2.is_finite() || !lon2.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let max_lat = core::f64::consts::FRAC_PI_2;
    if lat1.abs() > max_lat || lat2.abs() > max_lat {
        return Err(GeoError::InvalidLatitude);
    }
    Ok(())
}
