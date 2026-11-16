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

/// Madgwick AHRS 6-DOF orientation filter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MadgwickAhrs {
    /// Orientation represented as unit quaternion.
    pub quat: Quaternion,
    /// Gradient descent algorithm gain.
    pub beta: f32,
}

impl MadgwickAhrs {
    /// Default filter gain for IMU 6-DOF fusion.
    pub const DEFAULT_BETA: f32 = 0.033;
}
