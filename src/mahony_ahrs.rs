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

impl MahonyAhrs {
    /// Current Euler angles (roll, pitch, yaw) in radians.
    pub fn euler_zyx(&self) -> (f32, f32, f32) {
        self.quat.to_euler_zyx()
    }

    /// Current estimated 3-axis gyroscope bias in rad/s.
    pub const fn gyro_bias(&self) -> [f32; 3] {
        self.gyro_bias
    }

    /// Manually set the estimated 3-axis gyroscope bias.
    pub fn set_gyro_bias(&mut self, bias: [f32; 3]) {
        self.gyro_bias = bias;
    }
}

impl MahonyAhrs {
    /// Reset estimated gyroscope bias to zero.
    pub fn reset_gyro_bias(&mut self) {
        self.gyro_bias = [0.0; 3];
    }

    /// Reset filter state to identity quaternion and zero bias.
    pub fn reset(&mut self) {
        self.quat = Quaternion::IDENTITY;
        self.gyro_bias = [0.0; 3];
    }
}

impl MahonyAhrs {
    fn compute_gravity_error(q: &Quaternion, accel: [f32; 3]) -> [f32; 3] {
        let a_sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
        if a_sq < 1e-12 || !a_sq.is_finite() {
            return [0.0; 3];
        }
        let inv_a = 1.0 / libm::sqrtf(a_sq);
        let (ax, ay, az) = (accel[0] * inv_a, accel[1] * inv_a, accel[2] * inv_a);

        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let vx = 2.0 * (x * z - w * y);
        let vy = 2.0 * (w * x + y * z);
        let vz = 1.0 - 2.0 * (x * x + y * y);

        [ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx]
    }
}
