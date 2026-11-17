//! Earth gravity spherical harmonic WGS84 normal gravity calculator.

/// Errors occurring during WGS84 gravity evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wgs84GravityError {
    /// Non-finite coordinate or parameter encountered.
    NonFinite,
    /// Latitude outside valid range [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Altitude outside supported range [-10_000 m, 100_000 m].
    InvalidAltitude,
    /// Radius too small or singular (r < 1.0 m).
    SingularRadius,
}

/// WGS-84 semi-major axis in meters (a).
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 semi-minor axis in meters (b).
pub const WGS84_B: f64 = 6_356_752.314_245;

/// WGS-84 flattening (f = (a - b) / a).
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// WGS-84 first eccentricity squared (e^2).
pub const WGS84_E2: f64 = 0.006_694_379_990_14;

/// WGS-84 Earth gravitational constant (GM) in m^3/s^2.
pub const WGS84_GM: f64 = 3.986_004_418e14;

/// WGS-84 Earth angular velocity in rad/s.
pub const WGS84_OMEGA: f64 = 7.292_115e-5;

/// WGS-84 second zonal harmonic coefficient J2.
pub const WGS84_J2: f64 = 1.082_629_821_31e-3;

/// WGS-84 fourth zonal harmonic coefficient J4.
pub const WGS84_J4: f64 = -2.370_912_22e-6;

/// WGS-84 equatorial normal gravity in m/s^2 (gamma_e).
pub const GAMMA_E: f64 = 9.780_325_335_9;

/// WGS-84 polar normal gravity in m/s^2 (gamma_p).
pub const GAMMA_P: f64 = 9.832_184_937_8;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f64 = 0.001_931_852_652_41;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid in m/s^2.
pub fn somigliana_surface(lat_rad: f64) -> Result<f64, Wgs84GravityError> {
    if !lat_rad.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(Wgs84GravityError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let s2 = s * s;
    let num = 1.0 + SOMIGLIANA_K * s2;
    let den = libm::sqrt(1.0 - WGS84_E2 * s2);
    Ok(GAMMA_E * (num / den))
}

/// Somigliana normal gravity at surface in f32 for Cortex-M4F navigation.
pub fn somigliana_surface_f32(lat_rad: f32) -> Result<f32, Wgs84GravityError> {
    if !lat_rad.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let half_pi = core::f32::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(Wgs84GravityError::InvalidLatitude);
    }
    let s = libm::sinf(lat_rad);
    let s2 = s * s;
    let num = 1.0f32 + (SOMIGLIANA_K as f32) * s2;
    let den = libm::sqrtf(1.0f32 - (WGS84_E2 as f32) * s2);
    Ok((GAMMA_E as f32) * (num / den))
}
