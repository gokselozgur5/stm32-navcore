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
