use crate::attitude::AttitudeError;
use crate::math::Quaternion;

/// Mahony non-linear complementary filter with PI gyro bias correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mahony {
    pub q: Quaternion,
    pub kp: f32,
    pub ki: f32,
    pub gyro_bias: [f32; 3],
}

impl Mahony {
    /// Default proportional gain for accelerometer correction.
    pub const DEFAULT_KP: f32 = 2.0;

    /// Default integral gain for gyro bias correction.
    pub const DEFAULT_KI: f32 = 0.05;
}
