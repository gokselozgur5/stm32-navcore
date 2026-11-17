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

/// Compute Euler angle rates (roll_dot, pitch_dot, yaw_dot) from body angular rates (p, q, r).
pub fn body_rates_to_euler_rates(
    euler: EulerAngles,
    omega: [f32; 3],
) -> Result<[f32; 3], KinematicsError> {
    let m = body_to_euler_rates_matrix(euler)?;
    Ok([
        m[0][0] * omega[0] + m[0][1] * omega[1] + m[0][2] * omega[2],
        m[1][0] * omega[0] + m[1][1] * omega[1] + m[1][2] * omega[2],
        m[2][0] * omega[0] + m[2][1] * omega[1] + m[2][2] * omega[2],
    ])
}

/// Skew-symmetric 4x4 matrix Omega(omega) for quaternion kinematic propagation.
pub fn omega_matrix(omega: [f32; 3]) -> [[f32; 4]; 4] {
    let (wx, wy, wz) = (omega[0], omega[1], omega[2]);
    [
        [0.0, -wx, -wy, -wz],
        [wx, 0.0, wz, -wy],
        [wy, -wz, 0.0, wx],
        [wz, wy, -wx, 0.0],
    ]
}

/// Xi matrix (4x3) mapping body angular velocity to quaternion rate: q_dot = 0.5 * Xi(q) * omega.
pub fn xi_matrix(q: &Quaternion) -> [[f32; 3]; 4] {
    [
        [-q.x, -q.y, -q.z],
        [q.w, -q.z, q.y],
        [q.z, q.w, -q.x],
        [-q.y, q.x, q.w],
    ]
}

/// Compute quaternion derivative [w_dot, x_dot, y_dot, z_dot] from body rates.
pub fn quat_derivative(q: &Quaternion, omega: [f32; 3]) -> [f32; 4] {
    let xi = xi_matrix(q);
    [
        0.5 * (xi[0][0] * omega[0] + xi[0][1] * omega[1] + xi[0][2] * omega[2]),
        0.5 * (xi[1][0] * omega[0] + xi[1][1] * omega[1] + xi[1][2] * omega[2]),
        0.5 * (xi[2][0] * omega[0] + xi[2][1] * omega[1] + xi[2][2] * omega[2]),
        0.5 * (xi[3][0] * omega[0] + xi[3][1] * omega[1] + xi[3][2] * omega[2]),
    ]
}

/// Compute body rates from quaternion and its time derivative: omega = 2 * Xi(q)^T * q_dot.
pub fn quat_derivative_to_body_rates(q: &Quaternion, q_dot: [f32; 4]) -> [f32; 3] {
    let xi = xi_matrix(q);
    [
        2.0 * (xi[0][0] * q_dot[0] + xi[1][0] * q_dot[1] + xi[2][0] * q_dot[2] + xi[3][0] * q_dot[3]),
        2.0 * (xi[0][1] * q_dot[0] + xi[1][1] * q_dot[1] + xi[2][1] * q_dot[2] + xi[3][1] * q_dot[3]),
        2.0 * (xi[0][2] * q_dot[0] + xi[1][2] * q_dot[1] + xi[2][2] * q_dot[2] + xi[3][2] * q_dot[3]),
    ]
}

/// Convert quaternion rates to Euler angle rates given current attitude quaternion.
pub fn quat_rates_to_euler_rates(
    q: &Quaternion,
    q_dot: [f32; 4],
) -> Result<[f32; 3], KinematicsError> {
    let euler = EulerAngles::from_quat(q);
    let omega = quat_derivative_to_body_rates(q, q_dot);
    body_rates_to_euler_rates(euler, omega)
}

/// Convert Euler angle rates to quaternion rates given current Euler angles.
pub fn euler_rates_to_quat_rates(euler: EulerAngles, euler_rates: [f32; 3]) -> [f32; 4] {
    let q = euler.to_quat();
    let omega = euler_rates_to_body_rates(euler, euler_rates);
    quat_derivative(&q, omega)
}

fn phi_scales(norm: f32, half_angle: f32, dt: f32) -> (f32, f32) {
    if norm < 1e-7 {
        (1.0 - 0.5 * half_angle * half_angle, 0.5 * dt)
    } else {
        (libm::cosf(half_angle), libm::sinf(half_angle) / norm)
    }
}

/// Discrete quaternion state transition matrix Phi(dt) = exp(0.5 * Omega * dt).
pub fn attitude_transition_matrix(
    omega: [f32; 3],
    dt: f32,
) -> Result<[[f32; 4]; 4], KinematicsError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(KinematicsError::InvalidDeltaTime);
    }
    let (wx, wy, wz) = (omega[0], omega[1], omega[2]);
    if !wx.is_finite() || !wy.is_finite() || !wz.is_finite() {
        return Err(KinematicsError::NonFinite);
    }
    let norm = libm::sqrtf(wx * wx + wy * wy + wz * wz);
    let (c, s) = phi_scales(norm, 0.5 * norm * dt, dt);
    let om = omega_matrix(omega);
    let mut phi = [[0.0f32; 4]; 4];
    for r in 0..4 {
        for c_idx in 0..4 {
            let diag = if r == c_idx { c } else { 0.0 };
            phi[r][c_idx] = diag + s * om[r][c_idx];
        }
    }
    Ok(phi)
}

/// Propagate attitude quaternion forward by time step dt using exact transition matrix.
pub fn propagate_attitude_quaternion(
    q: &Quaternion,
    omega: [f32; 3],
    dt: f32,
) -> Result<Quaternion, KinematicsError> {
    let phi = attitude_transition_matrix(omega, dt)?;
    let q_vec = [q.w, q.x, q.y, q.z];
    let mut next = [0.0f32; 4];
    for r in 0..4 {
        let mut sum = 0.0;
        for c in 0..4 {
            sum += phi[r][c] * q_vec[c];
        }
        next[r] = sum;
    }
    let res = Quaternion::new(next[0], next[1], next[2], next[3]);
    res.normalize().map_err(|_| KinematicsError::ZeroQuaternion)
}

/// Propagate Euler angles forward by time step dt using Euler rate kinematics.
pub fn propagate_euler_angles(
    euler: EulerAngles,
    omega: [f32; 3],
    dt: f32,
) -> Result<EulerAngles, KinematicsError> {
    if !dt.is_finite() || dt <= 0.0 {
        return Err(KinematicsError::InvalidDeltaTime);
    }
    let rates = body_rates_to_euler_rates(euler, omega)?;
    Ok(EulerAngles {
        roll: euler.roll + rates[0] * dt,
        pitch: euler.pitch + rates[1] * dt,
        yaw: euler.yaw + rates[2] * dt,
    })
}
