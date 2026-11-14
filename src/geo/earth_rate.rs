//! Earth rotation rate and transport rate in NED.

use crate::geo::{GeoError, Ned, WGS84_OMEGA_EARTH};

/// Computes Earth rotation rate in NED at latitude $\phi$.
pub fn earth_rotation_rate(lat_rad: f64) -> Result<Ned, GeoError> {
    if !lat_rad.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(GeoError::InvalidLatitude);
    }
    let omega_n = WGS84_OMEGA_EARTH * libm::cos(lat_rad);
    let omega_d = -WGS84_OMEGA_EARTH * libm::sin(lat_rad);
    Ok(Ned::new(omega_n, 0.0, omega_d))
}

use crate::geo::{WGS84_A, WGS84_E2};

/// Computes meridian and normal radii of curvature $(R_M, R_N)$.
pub fn principal_radii_of_curvature(lat_rad: f64) -> Result<(f64, f64), GeoError> {
    if !lat_rad.is_finite() {
        return Err(GeoError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(GeoError::InvalidLatitude);
    }
    let sin_lat = libm::sin(lat_rad);
    let denom = 1.0 - WGS84_E2 * sin_lat * sin_lat;
    let sqrt_denom = libm::sqrt(denom);
    let r_n = WGS84_A / sqrt_denom;
    let r_m = WGS84_A * (1.0 - WGS84_E2) / (denom * sqrt_denom);
    Ok((r_m, r_n))
}
