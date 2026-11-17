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
