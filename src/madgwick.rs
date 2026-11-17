//! Madgwick AHRS sensor fusion filter for MARG and IMU configurations.

use crate::math::{MathError, Quaternion};

/// Error conditions for Madgwick AHRS operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MadgwickAhrsError {
    /// Non-positive or non-finite sampling period.
    InvalidDt,
    /// Non-positive or non-finite beta algorithm gain.
    InvalidBeta,
    /// Sensor measurement contains NaN or infinity.
    NonFinite,
    /// Quaternion normalization failed.
    NormalizationFailed,
}

impl From<MathError> for MadgwickAhrsError {
    fn from(_: MathError) -> Self {
        MadgwickAhrsError::NormalizationFailed
    }
}

/// Madgwick AHRS gradient descent orientation filter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MadgwickAhrs {
    /// Orientation represented as unit quaternion.
    pub quat: Quaternion,
    /// Filter gain representing convergence rate.
    pub beta: f32,
}

impl MadgwickAhrs {
    /// Default filter gain for IMU applications.
    pub const DEFAULT_BETA_IMU: f32 = 0.1;
    /// Default filter gain for MARG applications.
    pub const DEFAULT_BETA_MARG: f32 = 0.041;

    /// Create filter with given beta gain and identity attitude.
    pub fn new(beta: f32) -> Result<Self, MadgwickAhrsError> {
        if !beta.is_finite() || beta <= 0.0 {
            return Err(MadgwickAhrsError::InvalidBeta);
        }
        Ok(Self {
            quat: Quaternion::IDENTITY,
            beta,
        })
    }
}

impl MadgwickAhrs {
    /// Create filter with custom initial quaternion and beta gain.
    pub fn with_attitude(beta: f32, quat: Quaternion) -> Result<Self, MadgwickAhrsError> {
        if !beta.is_finite() || beta <= 0.0 {
            return Err(MadgwickAhrsError::InvalidBeta);
        }
        let q = quat.normalize()?;
        Ok(Self { quat: q, beta })
    }

    /// Current estimated orientation quaternion.
    pub const fn quaternion(&self) -> Quaternion {
        self.quat
    }

    /// Current filter beta gain.
    pub const fn beta(&self) -> f32 {
        self.beta
    }
}

impl MadgwickAhrs {
    /// Update beta gain value.
    pub fn set_beta(&mut self, beta: f32) -> Result<(), MadgwickAhrsError> {
        if !beta.is_finite() || beta <= 0.0 {
            return Err(MadgwickAhrsError::InvalidBeta);
        }
        self.beta = beta;
        Ok(())
    }

    /// Reset orientation quaternion to identity.
    pub fn reset(&mut self) {
        self.quat = Quaternion::IDENTITY;
    }

    /// Current Euler angles (roll, pitch, yaw) in radians.
    pub fn euler_zyx(&self) -> (f32, f32, f32) {
        self.quat.to_euler_zyx()
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

    fn accel_gradient(q: &Quaternion, a: [f32; 3]) -> [f32; 4] {
        let a_sq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
        if a_sq < 1e-12 || !a_sq.is_finite() {
            return [0.0; 4];
        }
        let inv_a = 1.0 / libm::sqrtf(a_sq);
        let (ax, ay, az) = (a[0] * inv_a, a[1] * inv_a, a[2] * inv_a);

        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let f1 = 2.0 * (x * z - w * y) - ax;
        let f2 = 2.0 * (w * x + y * z) - ay;
        let f3 = 1.0 - 2.0 * (x * x + y * y) - az;

        [
            -2.0 * y * f1 + 2.0 * x * f2,
            2.0 * z * f1 + 2.0 * w * f2 - 4.0 * x * f3,
            -2.0 * w * f1 + 2.0 * z * f2 - 4.0 * y * f3,
            2.0 * x * f1 + 2.0 * y * f2,
        ]
    }

    fn mag_flux(q: &Quaternion, m: [f32; 3]) -> Option<([f32; 3], f32, f32)> {
        let m_sq = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
        if m_sq < 1e-12 || !m_sq.is_finite() {
            return None;
        }
        let inv_m = 1.0 / libm::sqrtf(m_sq);
        let m_hat = [m[0] * inv_m, m[1] * inv_m, m[2] * inv_m];
        let h = q.rotate_vec(m_hat);
        let bx = libm::sqrtf(h[0] * h[0] + h[1] * h[1]);
        let bz = h[2];
        Some((m_hat, bx, bz))
    }

    fn mag_gradient(q: &Quaternion, m: [f32; 3]) -> [f32; 4] {
        let (m_hat, bx, bz) = match Self::mag_flux(q, m) {
            Some(res) => res,
            None => return [0.0; 4],
        };
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let f1 = 2.0 * bx * (0.5 - y * y - z * z) + 2.0 * bz * (x * z - w * y) - m_hat[0];
        let f2 = 2.0 * bx * (x * y - w * z) + 2.0 * bz * (w * x + y * z) - m_hat[1];
        let f3 = 2.0 * bx * (w * y + x * z) + 2.0 * bz * (0.5 - x * x - y * y) - m_hat[2];

        [
            -2.0 * bz * y * f1 + (-2.0 * bx * z + 2.0 * bz * x) * f2 + 2.0 * bx * y * f3,
            2.0 * bz * z * f1 + (2.0 * bx * y + 2.0 * bz * w) * f2 + (2.0 * bx * z - 4.0 * bz * x) * f3,
            (-4.0 * bx * y - 2.0 * bz * w) * f1 + (2.0 * bx * x + 2.0 * bz * z) * f2 + (2.0 * bx * w - 4.0 * bz * y) * f3,
            (-4.0 * bx * z + 2.0 * bz * x) * f1 + (-2.0 * bx * w + 2.0 * bz * y) * f2 + 2.0 * bx * x * f3,
        ]
    }
}

fn validate_imu(
    gyro: [f32; 3],
    accel: [f32; 3],
    beta: f32,
    dt: f32,
) -> Result<(), MadgwickAhrsError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(MadgwickAhrsError::InvalidDt);
    }
    if !beta.is_finite() || beta <= 0.0 {
        return Err(MadgwickAhrsError::InvalidBeta);
    }
    for &g in &gyro {
        if !g.is_finite() {
            return Err(MadgwickAhrsError::NonFinite);
        }
    }
    for &a in &accel {
        if !a.is_finite() {
            return Err(MadgwickAhrsError::NonFinite);
        }
    }
    Ok(())
}

fn validate_marg(
    gyro: [f32; 3],
    accel: [f32; 3],
    mag: [f32; 3],
    beta: f32,
    dt: f32,
) -> Result<(), MadgwickAhrsError> {
    validate_imu(gyro, accel, beta, dt)?;
    for &m in &mag {
        if !m.is_finite() {
            return Err(MadgwickAhrsError::NonFinite);
        }
    }
    Ok(())
}
