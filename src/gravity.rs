//! Somigliana normal gravity formula and elevation correction.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GravityError {
    NonFinite,
    InvalidLatitude,
    InvalidAltitude,
}

/// WGS-84 equatorial normal gravity in m/s^2.
pub const GAMMA_E: f32 = 9.780_325;

/// WGS-84 polar normal gravity in m/s^2.
pub const GAMMA_P: f32 = 9.832_185;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f32 = 0.001_931_853;

/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f32 = 0.006_694_38;

/// WGS-84 semi-major axis in meters.
pub const WGS84_A: f32 = 6_378_137.0;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid.
pub fn somigliana_surface(lat_rad: f32) -> Result<f32, GravityError> {
    if !lat_rad.is_finite() {
        return Err(GravityError::NonFinite);
    }
    let half_pi = core::f32::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(GravityError::InvalidLatitude);
    }
    let s = libm::sinf(lat_rad);
    let s2 = s * s;
    let num = 1.0 + SOMIGLIANA_K * s2;
    let den = libm::sqrtf(1.0 - WGS84_E2 * s2);
    Ok(GAMMA_E * (num / den))
}

/// Free-air elevation gravity correction in m/s^2.
pub fn elevation_correction(lat_rad: f32, alt_m: f32) -> Result<f32, GravityError> {
    if !alt_m.is_finite() {
        return Err(GravityError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(GravityError::InvalidAltitude);
    }
    let g0 = somigliana_surface(lat_rad)?;
    let f = 0.003_352_810_7;
    let m = 0.003_449_786_5;
    let s = libm::sinf(lat_rad);
    let s2 = s * s;
    let term1 = (2.0 / WGS84_A) * (1.0 + f + m - 2.0 * f * s2) * alt_m;
    let term2 = (3.0 / (WGS84_A * WGS84_A)) * alt_m * alt_m;
    Ok(-g0 * (term1 - term2))
}
