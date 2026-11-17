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
