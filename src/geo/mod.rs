//! Geodesy module: Earth reference models and coordinate transforms.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeoError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
}

/// WGS-84 semi-major axis (equatorial radius) in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 reciprocal flattening (1/f).
pub const WGS84_INV_F: f64 = 298.257_223_563;

/// WGS-84 flattening: f = 1 / 298.257223563.
pub const WGS84_F: f64 = 1.0 / WGS84_INV_F;

/// WGS-84 semi-minor axis (polar radius) in meters: b = a * (1 - f).
pub const WGS84_B: f64 = 6_356_752.314_245_179;

/// WGS-84 first eccentricity squared: e^2 = 2f - f^2.
pub const WGS84_E2: f64 = 0.006_694_379_990_141_316_5;

/// WGS-84 second eccentricity squared: e'^2 = (a^2 - b^2) / b^2.
pub const WGS84_E_PRIME2: f64 = 0.006_739_496_742_276_434;

/// WGS-84 Earth rotation rate in rad/s.
pub const WGS84_OMEGA_EARTH: f64 = 7.292_115e-5;

pub mod ecef;
pub use ecef::{lla_to_ecef, Ecef, Lla};
pub mod bowring;
pub use bowring::ecef_to_lla;
