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
