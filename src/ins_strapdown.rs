//! Deterministic INS strapdown integration and Coriolis compensator.

/// Errors occurring during strapdown mechanization and Coriolis compensation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InsStrapdownError {
    /// Non-finite coordinate, rate, or acceleration encountered.
    NonFinite,
    /// Latitude outside valid range [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Delta time dt is non-positive or non-finite.
    InvalidDeltaTime,
    /// Altitude outside supported range [-10_000 m, 100_000 m].
    InvalidAltitude,
    /// Quaternion norm is zero or degenerate.
    SingularQuaternion,
}

impl core::fmt::Display for InsStrapdownError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "non-finite input encountered"),
            Self::InvalidLatitude => write!(f, "latitude must be within [-pi/2, pi/2]"),
            Self::InvalidDeltaTime => write!(f, "delta time must be positive and finite"),
            Self::InvalidAltitude => write!(f, "altitude must be between -10000 and 100000 m"),
            Self::SingularQuaternion => write!(f, "attitude quaternion is singular"),
        }
    }
}

/// WGS-84 semi-major axis in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f64 = 0.006_694_379_990_14;

/// Earth rotation angular velocity in rad/s.
pub const EARTH_ROTATION_RATE: f64 = 7.292_115e-5;

/// Somigliana formula equatorial normal gravity in m/s^2.
pub const GAMMA_E: f64 = 9.780_325_335_9;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f64 = 0.001_931_852_652_41;
