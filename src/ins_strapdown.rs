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
