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

impl MahonyAhrs {
    fn compute_mag_error(q: &Quaternion, mag: [f32; 3]) -> [f32; 3] {
        let m_sq = mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2];
        if m_sq < 1e-12 || !m_sq.is_finite() {
            return [0.0; 3];
        }
        let inv_m = 1.0 / libm::sqrtf(m_sq);
        let (mx, my, mz) = (mag[0] * inv_m, mag[1] * inv_m, mag[2] * inv_m);

        let h = q.rotate_vec([mx, my, mz]);
        let bx = libm::sqrtf(h[0] * h[0] + h[1] * h[1]);
        let bz = h[2];
        let q_conj = Quaternion::new(q.w, -q.x, -q.y, -q.z);
        let w = q_conj.rotate_vec([bx, 0.0, bz]);

        [my * w[2] - mz * w[1], mz * w[0] - mx * w[2], mx * w[1] - my * w[0]]
    }
}

fn validate_6dof(gyro: [f32; 3], accel: [f32; 3], dt: f32) -> Result<(), MahonyAhrsError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(MahonyAhrsError::InvalidDt);
    }
    for &g in &gyro {
        if !g.is_finite() {
            return Err(MahonyAhrsError::NonFinite);
        }
    }
    for &a in &accel {
        if !a.is_finite() {
            return Err(MahonyAhrsError::NonFinite);
        }
    }
    Ok(())
}

fn validate_9dof(
    gyro: [f32; 3],
    accel: [f32; 3],
    mag: [f32; 3],
    dt: f32,
) -> Result<(), MahonyAhrsError> {
    validate_6dof(gyro, accel, dt)?;
    for &m in &mag {
        if !m.is_finite() {
            return Err(MahonyAhrsError::NonFinite);
        }
    }
    Ok(())
}

fn step_correction(
    gyro: [f32; 3],
    gyro_bias: [f32; 3],
    e: [f32; 3],
    kp: f32,
    ki: f32,
    dt: f32,
) -> ([f32; 3], [f32; 3]) {
    let ki_dt = ki * dt;
    let nb = [
        gyro_bias[0] - ki_dt * e[0],
        gyro_bias[1] - ki_dt * e[1],
        gyro_bias[2] - ki_dt * e[2],
    ];
    let w = [
        gyro[0] - nb[0] + kp * e[0],
        gyro[1] - nb[1] + kp * e[1],
        gyro[2] - nb[2] + kp * e[2],
    ];
    (nb, w)
}

/// Perform one 6-DOF Mahony AHRS filter step.
pub fn mahony_ahrs_6dof_step(
    q: Quaternion,
    gyro: [f32; 3],
    accel: [f32; 3],
    gyro_bias: [f32; 3],
    kp: f32,
    ki: f32,
    dt: f32,
) -> Result<(Quaternion, [f32; 3]), MahonyAhrsError> {
    validate_6dof(gyro, accel, dt)?;
    let e = MahonyAhrs::compute_gravity_error(&q, accel);
    let (next_bias, w) = step_correction(gyro, gyro_bias, e, kp, ki, dt);
    let next_q = q.integrate_gyro(w, dt)?;
    Ok((next_q, next_bias))
}

/// Perform one 9-DOF Mahony AHRS filter step with magnetometer correction.
pub fn mahony_ahrs_9dof_step(
    q: Quaternion,
    gyro: [f32; 3],
    accel: [f32; 3],
    mag: [f32; 3],
    gyro_bias: [f32; 3],
    kp: f32,
    ki: f32,
    dt: f32,
) -> Result<(Quaternion, [f32; 3]), MahonyAhrsError> {
    validate_9dof(gyro, accel, mag, dt)?;
    let ea = MahonyAhrs::compute_gravity_error(&q, accel);
    let em = MahonyAhrs::compute_mag_error(&q, mag);
    let e = [ea[0] + em[0], ea[1] + em[1], ea[2] + em[2]];
    let (next_bias, w) = step_correction(gyro, gyro_bias, e, kp, ki, dt);
    let next_q = q.integrate_gyro(w, dt)?;
    Ok((next_q, next_bias))
}

impl MahonyAhrs {
    /// Update orientation estimate using 6-DOF gyro and accelerometer.
    pub fn update(
        &mut self,
        gyro: [f32; 3],
        accel: [f32; 3],
        dt: f32,
    ) -> Result<Quaternion, MahonyAhrsError> {
        self.update_6dof(gyro, accel, dt)
    }

    /// Update orientation estimate using 6-DOF gyro and accelerometer.
    pub fn update_6dof(
        &mut self,
        gyro: [f32; 3],
        accel: [f32; 3],
        dt: f32,
    ) -> Result<Quaternion, MahonyAhrsError> {
        let (nq, nb) =
            mahony_ahrs_6dof_step(self.quat, gyro, accel, self.gyro_bias, self.kp, self.ki, dt)?;
        self.quat = nq;
        self.gyro_bias = nb;
        Ok(self.quat)
    }
}
