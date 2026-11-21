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
    if beta == 0.0 {
        let qw = [
            0.5 * (-q.x * gyro[0] - q.y * gyro[1] - q.z * gyro[2]),
            0.5 * (q.w * gyro[0] + q.y * gyro[2] - q.z * gyro[1]),
            0.5 * (q.w * gyro[1] - q.x * gyro[2] + q.z * gyro[0]),
            0.5 * (q.w * gyro[2] + q.x * gyro[1] - q.y * gyro[0]),
        ];
        let next = Quaternion::new(
            q.w + qw[0] * dt,
            q.x + qw[1] * dt,
            q.y + qw[2] * dt,
            q.z + qw[3] * dt,
        );
        return Ok(next.normalize()?);
    }
    crate::madgwick::madgwick_imu_step(q, gyro, accel, beta, dt).map_err(|e| match e {
        crate::madgwick::MadgwickAhrsError::InvalidDt => AhrsError::InvalidDt,
        crate::madgwick::MadgwickAhrsError::InvalidBeta => AhrsError::InvalidDt,
        crate::madgwick::MadgwickAhrsError::NonFinite => AhrsError::NonFinite,
        crate::madgwick::MadgwickAhrsError::NormalizationFailed => AhrsError::NormalizationFailed,
    })
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
