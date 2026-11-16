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
