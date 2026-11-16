//! NED to Body frame rotation matrix operations.

use crate::math::Mat;
use libm::{asinf, atan2f, cosf, sinf, sqrtf};

/// Error types for NED to Body rotations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyRotError {
    /// Non-finite coordinate, vector, or matrix component.
    NonFinite,
    /// Matrix failed orthonormality verification.
    NotOrthonormal,
    /// Gimbal lock condition in Euler conversion.
    GimbalLock,
}

/// Numerical tolerance for DCM orthonormality checks.
pub const BODY_ROT_TOLERANCE: f32 = 1e-4;
