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

impl Mahony {
    /// Create a Mahony filter with given gains and identity attitude.
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            q: Quaternion::IDENTITY,
            kp,
            ki,
            gyro_bias: [0.0; 3],
        }
    }

    /// Create a filter initialized with an existing orientation quaternion.
    pub fn with_attitude(kp: f32, ki: f32, q: Quaternion) -> Result<Self, AttitudeError> {
        let q_norm = q.normalize().map_err(|_| AttitudeError::InvalidInput)?;
        Ok(Self {
            q: q_norm,
            kp,
            ki,
            gyro_bias: [0.0; 3],
        })
    }
}

