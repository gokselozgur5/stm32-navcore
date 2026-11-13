//! Madgwick 6-axis gradient descent attitude estimator.

use crate::math::{MathError, Quaternion};

/// Madgwick IMU (6-axis gyro + accel) attitude estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Madgwick {
    pub quat: Quaternion,
    pub beta: f32,
}

impl Madgwick {
    pub const DEFAULT_BETA: f32 = 0.1;

    pub const fn new(beta: f32) -> Self {
        Self {
            quat: Quaternion::IDENTITY,
            beta,
        }
    }

    pub fn set_quaternion(&mut self, q: Quaternion) -> Result<(), MathError> {
        self.quat = q.normalize()?;
        Ok(())
    }

    pub fn reset(&mut self) {
        self.quat = Quaternion::IDENTITY;
    }

    pub fn quaternion(&self) -> Quaternion {
        self.quat
    }
}
