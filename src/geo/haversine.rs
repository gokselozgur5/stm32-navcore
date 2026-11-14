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

fn great_circle_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let s_lat = libm::sin(dlat * 0.5);
    let s_lon = libm::sin(dlon * 0.5);
    let a = s_lat * s_lat + libm::cos(lat1) * libm::cos(lat2) * s_lon * s_lon;
    let clamped_a = a.clamp(0.0, 1.0);
    2.0 * EARTH_RADIUS_M * libm::atan2(libm::sqrt(clamped_a), libm::sqrt(1.0 - clamped_a))
}

fn initial_bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlon = lon2 - lon1;
    let y = libm::sin(dlon) * libm::cos(lat2);
    let x = libm::cos(lat1) * libm::sin(lat2) - libm::sin(lat1) * libm::cos(lat2) * libm::cos(dlon);
    let az = libm::atan2(y, x);
    if az < 0.0 {
        az + 2.0 * core::f64::consts::PI
    } else {
        az
    }
}

/// Computes great-circle distance and initial bearing between two points in radians.
pub fn haversine(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> Result<HaversineResult, GeoError> {
    validate_coords(lat1, lon1, lat2, lon2)?;
    let distance_m = great_circle_distance(lat1, lon1, lat2, lon2);
    let bearing_rad = initial_bearing(lat1, lon1, lat2, lon2);
    Ok(HaversineResult {
        distance_m,
        bearing_rad,
    })
}
