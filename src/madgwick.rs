//! Madgwick AHRS sensor fusion filter for MARG and IMU configurations.

use crate::math::{MathError, Quaternion};

/// Error conditions for Madgwick AHRS operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MadgwickAhrsError {
    /// Non-positive or non-finite sampling period.
    InvalidDt,
    /// Non-positive or non-finite beta algorithm gain.
    InvalidBeta,
    /// Sensor measurement contains NaN or infinity.
    NonFinite,
    /// Quaternion normalization failed.
    NormalizationFailed,
}

impl From<MathError> for MadgwickAhrsError {
    fn from(_: MathError) -> Self {
        MadgwickAhrsError::NormalizationFailed
    }
}
