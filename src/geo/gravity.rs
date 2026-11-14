//! Somigliana normal gravity model with free-air correction.

use crate::geo::GeoError;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid.
pub fn normal_gravity_surface(lat_rad: f64) -> Result<f64, GeoError> {
    if !lat_rad.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(GeoError::InvalidLatitude);
    }
    let gamma_e = 9.780_325_335_9;
    let k = 0.001_931_852_652_41;
    let e2 = 0.006_694_379_990_14;
    let s = libm::sin(lat_rad);
    let s2 = s * s;
    let num = 1.0 + k * s2;
    let den = libm::sqrt(1.0 - e2 * s2);
    Ok(gamma_e * (num / den))
}

/// Somigliana normal gravity with free-air height correction.
pub fn normal_gravity(lat_rad: f64, height_m: f64) -> Result<f64, GeoError> {
    if !height_m.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let g0 = normal_gravity_surface(lat_rad)?;
    let a = crate::geo::WGS84_A;
    let f = crate::geo::WGS84_F;
    let m = 0.003_449_786_506_84;
    let s = libm::sin(lat_rad);
    let s2 = s * s;
    let term1 = (2.0 / a) * (1.0 + f + m - 2.0 * f * s2) * height_m;
    let term2 = (3.0 / (a * a)) * height_m * height_m;
    Ok(g0 * (1.0 - term1 + term2))
}
