//! Madgwick 6-axis gradient descent attitude estimator.
//!
//! Accelerometer measurements follow the NED specific-force convention: a level
//! sensor at rest measures `[0, 0, -g]`.

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

impl Madgwick {
    pub fn update(
        &mut self,
        gyro_rad_s: [f32; 3],
        accel: [f32; 3],
        dt_s: f32,
    ) -> Result<Quaternion, MathError> {
        let q_next =
            crate::madgwick::madgwick_imu_step(self.quat, gyro_rad_s, accel, self.beta, dt_s)
                .map_err(|_| MathError::InvalidNorm)?;
        self.quat = q_next;
        Ok(self.quat)
    }
}

impl Default for Madgwick {
    fn default() -> Self {
        Self::new(Self::DEFAULT_BETA)
    }
}
