//! Mahony AHRS complementary filter with integral gyro bias tracking.

use crate::math::{MathError, Quaternion};

/// Error conditions for Mahony AHRS operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MahonyAhrsError {
    /// Non-positive or non-finite sampling period.
    InvalidDt,
    /// Sensor measurement contains NaN or infinity.
    NonFinite,
    /// Quaternion normalization failed.
    NormalizationFailed,
}

impl From<MathError> for MahonyAhrsError {
    fn from(_: MathError) -> Self {
        MahonyAhrsError::NormalizationFailed
    }
}

/// Mahony non-linear complementary filter with integral gyro bias tracking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MahonyAhrs {
    /// Orientation represented as unit quaternion.
    pub quat: Quaternion,
    /// Proportional feedback gain.
    pub kp: f32,
    /// Integral feedback gain for gyro bias tracking.
    pub ki: f32,
    /// Estimated gyroscope bias in rad/s.
    pub gyro_bias: [f32; 3],
}
