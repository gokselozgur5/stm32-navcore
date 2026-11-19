use crate::geo::GeoError;
use core::f32::consts::PI;

/// Computes the initial great-circle bearing from (lat1, lon1) to (lat2, lon2) in radians [0, 2*PI).
pub fn initial_bearing_rad(
    lat1_rad: f32,
    lon1_rad: f32,
    lat2_rad: f32,
    lon2_rad: f32,
) -> Result<f32, GeoError> {
    if !lat1_rad.is_finite()
        || !lon1_rad.is_finite()
        || !lat2_rad.is_finite()
        || !lon2_rad.is_finite()
    {
        return Err(GeoError::NonFinite);
    }
    let dlon = lon2_rad - lon1_rad;
    let y = libm::sinf(dlon) * libm::cosf(lat2_rad);
    let x = libm::cosf(lat1_rad) * libm::sinf(lat2_rad)
        - libm::sinf(lat1_rad) * libm::cosf(lat2_rad) * libm::cosf(dlon);
    let mut brng = libm::atan2f(y, x);
    if brng < 0.0 {
        brng += 2.0 * PI;
    }
    if brng >= 2.0 * PI {
        brng = 0.0;
    }
    Ok(brng)
}

/// Computes the initial great-circle bearing from (lat1, lon1) to (lat2, lon2) in degrees [0.0, 360.0).
pub fn initial_bearing_deg(
    lat1_deg: f32,
    lon1_deg: f32,
    lat2_deg: f32,
    lon2_deg: f32,
) -> Result<f32, GeoError> {
    if !lat1_deg.is_finite()
        || !lon1_deg.is_finite()
        || !lat2_deg.is_finite()
        || !lon2_deg.is_finite()
    {
        return Err(GeoError::NonFinite);
    }
    if lat1_deg < -90.0 || lat1_deg > 90.0 || lat2_deg < -90.0 || lat2_deg > 90.0 {
        return Err(GeoError::InvalidLatitude);
    }
    let r1 = lat1_deg * (PI / 180.0);
    let r2 = lon1_deg * (PI / 180.0);
    let r3 = lat2_deg * (PI / 180.0);
    let r4 = lon2_deg * (PI / 180.0);
    let brng_rad = initial_bearing_rad(r1, r2, r3, r4)?;
    let deg = brng_rad * (180.0 / PI);
    Ok(if deg >= 360.0 { 0.0 } else { deg })
}
