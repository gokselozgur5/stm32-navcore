//! Madgwick AHRS 6-DOF sensor fusion implementation.

use crate::math::{MathError, Quaternion};

/// Error conditions for AHRS operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AhrsError {
    /// Non-positive or non-finite sampling period.
    InvalidDt,
    /// Sensor measurement contains NaN or infinity.
    NonFinite,
    /// Quaternion normalization failed.
    NormalizationFailed,
}

impl From<MathError> for AhrsError {
    fn from(_: MathError) -> Self {
        AhrsError::NormalizationFailed
    }
}
