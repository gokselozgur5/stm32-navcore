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
