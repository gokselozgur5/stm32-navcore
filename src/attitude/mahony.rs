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

    /// Compute gravity error vector from normalized accelerometer and estimated attitude.
    fn compute_error(&self, a_unit: [f32; 3]) -> [f32; 3] {
        let (w, x, y, z) = (self.q.w, self.q.x, self.q.y, self.q.z);
        let vx = 2.0 * (w * y - x * z);
        let vy = -2.0 * (w * x + y * z);
        let vz = 2.0 * (x * x + y * y) - 1.0;
        [
            a_unit[1] * vz - a_unit[2] * vy,
            a_unit[2] * vx - a_unit[0] * vz,
            a_unit[0] * vy - a_unit[1] * vx,
        ]
    }
}

fn validate_sample(gyro: [f32; 3], accel: [f32; 3], dt: f32) -> Result<(), AttitudeError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(AttitudeError::InvalidInput);
    }
    for &v in gyro.iter().chain(accel.iter()) {
        if !v.is_finite() {
            return Err(AttitudeError::NonFinite);
        }
    }
    Ok(())
}

impl Mahony {
    fn accel_error(&self, accel: [f32; 3]) -> [f32; 3] {
        let sq = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
        if sq <= 1e-12 {
            return [0.0; 3];
        }
        let inv = 1.0 / libm::sqrtf(sq);
        self.compute_error([accel[0] * inv, accel[1] * inv, accel[2] * inv])
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
        validate_sample(gyro, accel, dt)?;
        let e = self.accel_error(accel);
        let ki_dt = self.ki * dt;
        self.gyro_bias[0] -= ki_dt * e[0];
        self.gyro_bias[1] -= ki_dt * e[1];
        self.gyro_bias[2] -= ki_dt * e[2];
        let w = [
            gyro[0] - self.gyro_bias[0] + self.kp * e[0],
            gyro[1] - self.gyro_bias[1] + self.kp * e[1],
            gyro[2] - self.gyro_bias[2] + self.kp * e[2],
        ];
        self.q = self
            .q
            .integrate_gyro(w, dt)
            .map_err(|_| AttitudeError::InvalidInput)?;
        Ok(())
    }
}
