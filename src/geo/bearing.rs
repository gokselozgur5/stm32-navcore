use core::f64::consts::PI;
use crate::geo::GeoError;

/// Computes the initial great-circle bearing from (lat1, lon1) to (lat2, lon2) in radians [0, 2*PI).
pub fn initial_bearing_rad(lat1_rad: f64, lon1_rad: f64, lat2_rad: f64, lon2_rad: f64) -> Result<f64, GeoError> {
    if !lat1_rad.is_finite() || !lon1_rad.is_finite() || !lat2_rad.is_finite() || !lon2_rad.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let dlon = lon2_rad - lon1_rad;
    let y = libm::sin(dlon) * libm::cos(lat2_rad);
    let x = libm::cos(lat1_rad) * libm::sin(lat2_rad)
        - libm::sin(lat1_rad) * libm::cos(lat2_rad) * libm::cos(dlon);
    let mut brng = libm::atan2(y, x);
    if brng < 0.0 {
        brng += 2.0 * PI;
    }
    Ok(brng)
}
