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
