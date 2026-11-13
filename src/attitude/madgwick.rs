//! Madgwick 6-axis gradient descent attitude estimator.

use crate::math::Quaternion;

/// Madgwick IMU (6-axis gyro + accel) attitude estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Madgwick {
    pub quat: Quaternion,
    pub beta: f32,
}
