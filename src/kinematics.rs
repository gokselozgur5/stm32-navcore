//! Attitude kinematic propagation and coordinate transform matrices.

use crate::math::Quaternion;

/// Errors arising in attitude kinematic transformations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum KinematicsError {
    /// Gimbal lock detected when pitch is close to +/-90 degrees.
    GimbalLock { pitch_rad: f32 },
    /// Time delta must be finite and positive.
    InvalidDeltaTime,
    /// Quaternion norm is zero or non-finite.
    ZeroQuaternion,
    /// Non-finite input or state encountered.
    NonFinite,
}

impl core::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}
