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
