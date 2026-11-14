//! Spherical linear and normalized linear interpolation for attitude quaternions.

use crate::attitude::AttitudeError;
use crate::math::Quaternion;

fn is_valid_unit(q: &Quaternion) -> bool {
    let f = [q.w, q.x, q.y, q.z];
    if f.iter().any(|v| !v.is_finite()) {
        return false;
    }
    let norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    (norm_sq - 1.0).abs() < 1e-4
}

fn dot(a: &Quaternion, b: &Quaternion) -> f32 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

/// Normalized linear interpolation between two attitude quaternions along the shortest path.
pub fn attitude_nlerp(q0: &Quaternion, q1: &Quaternion, t: f32) -> Result<Quaternion, AttitudeError> {
    if !t.is_finite() || !(0.0..=1.0).contains(&t) {
        return Err(AttitudeError::InvalidInput);
    }
    if !is_valid_unit(q0) || !is_valid_unit(q1) {
        return Err(AttitudeError::InvalidInput);
    }
    let s = if dot(q0, q1) < 0.0 { -1.0 } else { 1.0 };
    let q = Quaternion::new(
        q0.w + t * (s * q1.w - q0.w),
        q0.x + t * (s * q1.x - q0.x),
        q0.y + t * (s * q1.y - q0.y),
        q0.z + t * (s * q1.z - q0.z),
    );
    q.normalize().map_err(|_| AttitudeError::InvalidInput)
}

fn slerp_weights(cos_theta: f32, s: f32, t: f32) -> (f32, f32) {
    let theta = libm::acosf(cos_theta);
    let inv_sin = 1.0 / libm::sinf(theta);
    (libm::sinf((1.0 - t) * theta) * inv_sin, s * libm::sinf(t * theta) * inv_sin)
}
