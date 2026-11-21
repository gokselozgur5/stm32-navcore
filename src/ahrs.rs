//! Madgwick AHRS 6-DOF sensor fusion implementation.
//!
//! Accelerometer measurements follow the NED specific-force convention: a level
//! sensor at rest measures `[0, 0, -g]`.

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
    pub const DEFAULT_BETA: f32 = 0.1;
}

impl MadgwickAhrs {
    /// Create filter with given beta gain and identity orientation.
    pub const fn new(beta: f32) -> Self {
        Self {
            quat: Quaternion::IDENTITY,
            beta,
        }
    }

    /// Create filter with given beta gain and initial orientation quaternion.
    pub fn with_attitude(beta: f32, quat: Quaternion) -> Result<Self, AhrsError> {
        let q = quat.normalize()?;
        Ok(Self { quat: q, beta })
    }
}

impl MadgwickAhrs {
    /// Current estimated orientation quaternion.
    pub const fn quaternion(&self) -> Quaternion {
        self.quat
    }

    /// Current Euler angles (roll, pitch, yaw) in radians.
    pub fn euler_zyx(&self) -> (f32, f32, f32) {
        self.quat.to_euler_zyx()
    }

    /// Reset filter state to identity quaternion.
    pub fn reset(&mut self) {
        self.quat = Quaternion::IDENTITY;
    }
}

impl MadgwickAhrs {
    fn accel_gradient(q: &Quaternion, a: [f32; 3]) -> Option<[f32; 4]> {
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let f1 = 2.0 * (w * y - x * z) - a[0];
        let f2 = -2.0 * (w * x + y * z) - a[1];
        let f3 = 2.0 * (x * x + y * y) - 1.0 - a[2];

        let gw = 2.0 * y * f1 - 2.0 * x * f2;
        let gx = -2.0 * z * f1 - 2.0 * w * f2 + 4.0 * x * f3;
        let gy = 2.0 * w * f1 - 2.0 * z * f2 + 4.0 * y * f3;
        let gz = -2.0 * x * f1 - 2.0 * y * f2;

        let norm_sq = gw * gw + gx * gx + gy * gy + gz * gz;
        if norm_sq < 1e-12 || !norm_sq.is_finite() {
            return None;
        }
        let inv_norm = 1.0 / libm::sqrtf(norm_sq);
        Some([gw * inv_norm, gx * inv_norm, gy * inv_norm, gz * inv_norm])
    }
}

impl MadgwickAhrs {
    fn gyro_rate(q: &Quaternion, w: [f32; 3]) -> [f32; 4] {
        [
            0.5 * (-q.x * w[0] - q.y * w[1] - q.z * w[2]),
            0.5 * (q.w * w[0] + q.y * w[2] - q.z * w[1]),
            0.5 * (q.w * w[1] - q.x * w[2] + q.z * w[0]),
            0.5 * (q.w * w[2] + q.x * w[1] - q.y * w[0]),
        ]
    }
}

fn validate_inputs(gyro: [f32; 3], accel: [f32; 3], dt: f32) -> Result<(), AhrsError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(AhrsError::InvalidDt);
    }
    for &g in &gyro {
        if !g.is_finite() {
            return Err(AhrsError::NonFinite);
        }
    }
    for &a in &accel {
        if !a.is_finite() {
            return Err(AhrsError::NonFinite);
        }
    }
    Ok(())
}

/// Perform one 6-DOF Madgwick AHRS sensor fusion integration step.
pub fn madgwick_ahrs_6dof_step(
    q: Quaternion,
    gyro: [f32; 3],
    accel: [f32; 3],
    beta: f32,
    dt: f32,
) -> Result<Quaternion, AhrsError> {
    validate_inputs(gyro, accel, dt)?;
    let qw = MadgwickAhrs::gyro_rate(&q, gyro);
    let a_sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
    let g = if a_sq > 1e-12 {
        let inv = 1.0 / libm::sqrtf(a_sq);
        let ah = [accel[0] * inv, accel[1] * inv, accel[2] * inv];
        MadgwickAhrs::accel_gradient(&q, ah).unwrap_or([0.0; 4])
    } else {
        [0.0; 4]
    };
    let next = Quaternion::new(
        q.w + (qw[0] - beta * g[0]) * dt,
        q.x + (qw[1] - beta * g[1]) * dt,
        q.y + (qw[2] - beta * g[2]) * dt,
        q.z + (qw[3] - beta * g[3]) * dt,
    );
    Ok(next.normalize()?)
}

impl MadgwickAhrs {
    /// Update orientation estimate using angular velocity and linear acceleration.
    pub fn update(
        &mut self,
        gyro: [f32; 3],
        accel: [f32; 3],
        dt: f32,
    ) -> Result<Quaternion, AhrsError> {
        let q_next = madgwick_ahrs_6dof_step(self.quat, gyro, accel, self.beta, dt)?;
        self.quat = q_next;
        Ok(self.quat)
    }
}

impl Default for MadgwickAhrs {
    fn default() -> Self {
        Self::new(Self::DEFAULT_BETA)
    }
}
