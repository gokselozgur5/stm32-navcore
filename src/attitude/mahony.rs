use crate::attitude::AttitudeError;
use crate::math::Quaternion;

/// Mahony non-linear complementary filter with PI gyro bias correction.
///
/// Accelerometer measurements follow the NED specific-force convention: a level
/// sensor at rest measures `[0, 0, -g]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mahony {
    pub q: Quaternion,
    pub kp: f32,
    pub ki: f32,
    pub gyro_bias: [f32; 3],
}

impl Mahony {
    /// Default proportional gain for accelerometer correction.
    pub const DEFAULT_KP: f32 = 1.0;

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

impl Default for Mahony {
    fn default() -> Self {
        Self::new(Self::DEFAULT_KP, Self::DEFAULT_KI)
    }
}

impl Mahony {
    /// Current attitude estimate as a unit quaternion.
    pub fn quaternion(&self) -> Quaternion {
        self.q
    }

    /// Current attitude estimate as Tait-Bryan ZYX Euler angles (roll, pitch, yaw).
    pub fn euler_angles_zyx(&self) -> (f32, f32, f32) {
        self.q.to_euler_zyx()
    }

    /// Estimated 3-axis gyroscope bias in rad/s.
    pub fn gyro_bias(&self) -> [f32; 3] {
        self.gyro_bias
    }
}

impl Mahony {
    /// Manually set the estimated 3-axis gyroscope bias.
    pub fn set_gyro_bias(&mut self, bias: [f32; 3]) {
        self.gyro_bias = bias;
    }

    /// Reset estimated gyroscope bias to zero.
    pub fn reset_gyro_bias(&mut self) {
        self.gyro_bias = [0.0; 3];
    }
}

impl Mahony {
    /// Update attitude estimate using 3-axis gyro and 3-axis accelerometer.
    pub fn update(
        &mut self,
        gyro: [f32; 3],
        accel: [f32; 3],
        dt: f32,
    ) -> Result<(), AttitudeError> {
        let (nq, nb) = crate::mahony_ahrs::mahony_ahrs_6dof_step(
            self.q,
            gyro,
            accel,
            self.gyro_bias,
            self.kp,
            self.ki,
            dt,
        )
        .map_err(|e| match e {
            crate::mahony_ahrs::MahonyAhrsError::InvalidDt => AttitudeError::InvalidInput,
            crate::mahony_ahrs::MahonyAhrsError::NonFinite => AttitudeError::NonFinite,
            crate::mahony_ahrs::MahonyAhrsError::NormalizationFailed => AttitudeError::InvalidInput,
        })?;
        self.q = nq;
        self.gyro_bias = nb;
        Ok(())
    }
}
