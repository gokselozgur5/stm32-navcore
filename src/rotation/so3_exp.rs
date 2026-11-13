//! Exponential and logarithm maps between rotation vectors and quaternions.

use crate::math::{MathError, Quaternion};

/// Rotation vector (axis times angle, radians) to a unit quaternion.
pub fn so3_exp(v: [f32; 3]) -> Quaternion {
    let theta = libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    let k = if theta < 1e-4 {
        0.5 - theta * theta / 48.0
    } else {
        libm::sinf(0.5 * theta) / theta
    };
    Quaternion::new(libm::cosf(0.5 * theta), v[0] * k, v[1] * k, v[2] * k)
}

/// Unit quaternion to the rotation vector with angle in `[0, pi]`.
pub fn so3_log(q: &Quaternion) -> Result<[f32; 3], MathError> {
    let mut q = q.normalize()?;
    if q.w < 0.0 {
        q = Quaternion::new(-q.w, -q.x, -q.y, -q.z);
    }
    let vee = libm::sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    let k = if vee < 1e-4 {
        2.0 / q.w * (1.0 - vee * vee / (3.0 * q.w * q.w))
    } else {
        2.0 * libm::atan2f(vee, q.w) / vee
    };
    Ok([q.x * k, q.y * k, q.z * k])
}
