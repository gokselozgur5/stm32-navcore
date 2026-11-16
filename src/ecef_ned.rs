//! Geodetic, ECEF, and local tangent plane NED coordinate transforms.
//!
//! Provides deterministic, zero-allocation transformations using WGS-84.

/// Errors occurring during coordinate conversion or validation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EcefNedError {
    /// Coordinate contains NaN or infinite values.
    NonFinite,
    /// Geodetic latitude outside [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Geodetic longitude outside [-pi, pi] radians.
    InvalidLongitude,
    /// Position is too close to the Earth's center for inverse projection.
    SingularPoint,
}

impl core::fmt::Display for EcefNedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "coordinate values must be finite"),
            Self::InvalidLatitude => write!(f, "latitude must be within [-pi/2, pi/2]"),
            Self::InvalidLongitude => write!(f, "longitude must be within [-pi, pi]"),
            Self::SingularPoint => write!(f, "position is singular or inside inner core"),
        }
    }
}

/// Geodetic coordinates represented by latitude, longitude, and height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPos {
    /// Geodetic latitude in radians [-pi/2, pi/2].
    pub lat_rad: f64,
    /// Geodetic longitude in radians [-pi, pi].
    pub lon_rad: f64,
    /// Height above reference ellipsoid in meters.
    pub alt_m: f64,
}
