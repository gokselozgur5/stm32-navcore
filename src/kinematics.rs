//! Attitude kinematic propagation and coordinate transform matrices.

use crate::math::Quaternion;

/// Errors arising in attitude kinematic transformations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum KinematicsError {
    /// Gimbal lock detected when pitch is close to +/-90 degrees.
    GimbalLock { pitch_rad: f32 },
    /// Time delta must be finite and positive.
    InvalidDeltaTime,
    /// Quaternion norm is zero or non-finite.
    ZeroQuaternion,
    /// Non-finite input or state encountered.
    NonFinite,
}

impl core::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// Euler angles representation (Roll-Pitch-Yaw in radians).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EulerAngles {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl EulerAngles {
    pub const ZERO: Self = Self {
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    };

    pub const fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn to_array(&self) -> [f32; 3] {
        [self.roll, self.pitch, self.yaw]
    }
}

impl EulerAngles {
    /// Convert a unit quaternion to Euler angles (ZYX convention: roll, pitch, yaw).
    pub fn from_quat(q: &Quaternion) -> Self {
        let (r, p, y) = q.to_euler_zyx();
        Self {
            roll: r,
            pitch: p,
            yaw: y,
        }
    }

    /// Convert Euler angles (roll, pitch, yaw) to unit quaternion.
    pub fn to_quat(&self) -> Quaternion {
        let cr = libm::cosf(self.roll * 0.5);
        let sr = libm::sinf(self.roll * 0.5);
        let cp = libm::cosf(self.pitch * 0.5);
        let sp = libm::sinf(self.pitch * 0.5);
        let cy = libm::cosf(self.yaw * 0.5);
        let sy = libm::sinf(self.yaw * 0.5);

        Quaternion::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    }
}

/// Threshold on |cos(pitch)| below which kinematic mapping becomes singular.
pub const GIMBAL_LOCK_COS_THRESHOLD: f32 = 1e-3;

impl EulerAngles {
    /// Check whether the current attitude is in gimbal lock (|cos(pitch)| < threshold).
    pub fn is_gimbal_locked(&self) -> bool {
        libm::fabsf(libm::cosf(self.pitch)) < GIMBAL_LOCK_COS_THRESHOLD
    }
}

/// Transformation matrix mapping Euler rates (roll_dot, pitch_dot, yaw_dot) to body rates (p, q, r).
pub fn euler_rates_to_body_matrix(euler: EulerAngles) -> [[f32; 3]; 3] {
    let s_phi = libm::sinf(euler.roll);
    let c_phi = libm::cosf(euler.roll);
    let s_theta = libm::sinf(euler.pitch);
    let c_theta = libm::cosf(euler.pitch);

    [
        [1.0, 0.0, -s_theta],
        [0.0, c_phi, s_phi * c_theta],
        [0.0, -s_phi, c_phi * c_theta],
    ]
}

/// Transformation matrix mapping body rates (p, q, r) to Euler rates (roll_dot, pitch_dot, yaw_dot).
pub fn body_to_euler_rates_matrix(euler: EulerAngles) -> Result<[[f32; 3]; 3], KinematicsError> {
    let c_theta = libm::cosf(euler.pitch);
    if libm::fabsf(c_theta) < GIMBAL_LOCK_COS_THRESHOLD {
        return Err(KinematicsError::GimbalLock {
            pitch_rad: euler.pitch,
        });
    }
    let s_phi = libm::sinf(euler.roll);
    let c_phi = libm::cosf(euler.roll);
    let tan_theta = libm::tanf(euler.pitch);
    let sec_theta = 1.0 / c_theta;

    Ok([
        [1.0, s_phi * tan_theta, c_phi * tan_theta],
        [0.0, c_phi, -s_phi],
        [0.0, s_phi * sec_theta, c_phi * sec_theta],
    ])
}

/// Compute body angular rates (p, q, r) from Euler angles and Euler angle rates.
pub fn euler_rates_to_body_rates(euler: EulerAngles, rates: [f32; 3]) -> [f32; 3] {
    let m = euler_rates_to_body_matrix(euler);
    [
        m[0][0] * rates[0] + m[0][1] * rates[1] + m[0][2] * rates[2],
        m[1][0] * rates[0] + m[1][1] * rates[1] + m[1][2] * rates[2],
        m[2][0] * rates[0] + m[2][1] * rates[1] + m[2][2] * rates[2],
    ]
}
