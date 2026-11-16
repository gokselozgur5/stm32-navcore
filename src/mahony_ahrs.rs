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

impl MahonyAhrs {
    /// Default proportional feedback gain.
    pub const DEFAULT_KP: f32 = 1.0;
    /// Default integral feedback gain for gyro bias estimation.
    pub const DEFAULT_KI: f32 = 0.05;

    /// Create filter with given gains and identity attitude.
    pub const fn new(kp: f32, ki: f32) -> Self {
        Self {
            quat: Quaternion::IDENTITY,
            kp,
            ki,
            gyro_bias: [0.0; 3],
        }
    }
}

impl MahonyAhrs {
    /// Create filter with given gains and initial orientation quaternion.
    pub fn with_attitude(kp: f32, ki: f32, quat: Quaternion) -> Result<Self, MahonyAhrsError> {
        let q = quat.normalize()?;
        Ok(Self {
            quat: q,
            kp,
            ki,
            gyro_bias: [0.0; 3],
        })
    }

    /// Current estimated orientation quaternion.
    pub const fn quaternion(&self) -> Quaternion {
        self.quat
    }
}
