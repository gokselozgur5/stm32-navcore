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

impl Madgwick {
    fn gyro_rate(q: &Quaternion, w: [f32; 3]) -> [f32; 4] {
        [
            0.5 * (-q.x * w[0] - q.y * w[1] - q.z * w[2]),
            0.5 * (q.w * w[0] + q.y * w[2] - q.z * w[1]),
            0.5 * (q.w * w[1] - q.x * w[2] + q.z * w[0]),
            0.5 * (q.w * w[2] + q.x * w[1] - q.y * w[0]),
        ]
    }

    fn accel_gradient_step(q: &Quaternion, accel: [f32; 3], beta: f32) -> Option<[f32; 4]> {
        let a_norm = libm::sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
        if a_norm <= 1e-6 || !a_norm.is_finite() {
            return None;
        }
        let inv = 1.0 / a_norm;
        let a_hat = [accel[0] * inv, accel[1] * inv, accel[2] * inv];
        let g = Self::gradient(q, a_hat)?;
        Some([beta * g[0], beta * g[1], beta * g[2], beta * g[3]])
    }
}

impl Madgwick {
    pub fn update(
        &mut self,
        gyro_rad_s: [f32; 3],
        accel: [f32; 3],
        dt_s: f32,
    ) -> Result<Quaternion, MathError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(MathError::InvalidNorm);
        }
        let (wx, wy, wz) = (gyro_rad_s[0], gyro_rad_s[1], gyro_rad_s[2]);
        if !wx.is_finite() || !wy.is_finite() || !wz.is_finite() {
            return Err(MathError::InvalidNorm);
        }
        let q = self.quat;
        let q_dot = Self::gyro_rate(&q, gyro_rad_s);
        let grad = Self::accel_gradient_step(&q, accel, self.beta).unwrap_or([0.0; 4]);
        let next = Quaternion::new(
            q.w + (q_dot[0] - grad[0]) * dt_s,
            q.x + (q_dot[1] - grad[1]) * dt_s,
            q.y + (q_dot[2] - grad[2]) * dt_s,
            q.z + (q_dot[3] - grad[3]) * dt_s,
        );
        self.quat = next.normalize()?;
        Ok(self.quat)
    }
}

impl Default for Madgwick {
    fn default() -> Self {
        Self::new(Self::DEFAULT_BETA)
    }
}
