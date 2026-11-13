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

impl Madgwick {
    fn gradient(q: &Quaternion, a: [f32; 3]) -> Option<[f32; 4]> {
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let f1 = 2.0 * (x * z - w * y) - a[0];
        let f2 = 2.0 * (w * x + y * z) - a[1];
        let f3 = 1.0 - 2.0 * (x * x + y * y) - a[2];

        let gw = -2.0 * y * f1 + 2.0 * x * f2;
        let gx = 2.0 * z * f1 + 2.0 * w * f2 - 4.0 * x * f3;
        let gy = -2.0 * w * f1 + 2.0 * z * f2 - 4.0 * y * f3;
        let gz = 2.0 * x * f1 + 2.0 * y * f2;

        let norm = libm::sqrtf(gw * gw + gx * gx + gy * gy + gz * gz);
        if norm < 1e-9 || !norm.is_finite() {
            None
        } else {
            let inv = 1.0 / norm;
            Some([gw * inv, gx * inv, gy * inv, gz * inv])
        }
    }
}
