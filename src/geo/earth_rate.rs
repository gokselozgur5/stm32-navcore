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
