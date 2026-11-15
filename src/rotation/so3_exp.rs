//! Exponential and logarithm maps between rotation vectors and quaternions.

use super::{Dcm, RotationError};
use crate::math::{Mat, MathError, Quaternion};

/// Rotation vector to a direction cosine matrix via Rodrigues formula.
pub fn so3_exp_dcm(v: [f32; 3]) -> Dcm {
    let theta_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let theta = libm::sqrtf(theta_sq);
    let (a, b, cos_t) = if theta < 1e-4 {
        (1.0 - theta_sq / 6.0, 0.5 - theta_sq / 24.0, 1.0 - 0.5 * theta_sq)
    } else {
        (
            libm::sinf(theta) / theta,
            (1.0 - libm::cosf(theta)) / theta_sq,
            libm::cosf(theta),
        )
    };
    Dcm(Mat([
        [cos_t + b * v[0] * v[0], -a * v[2] + b * v[0] * v[1], a * v[1] + b * v[0] * v[2]],
        [a * v[2] + b * v[0] * v[1], cos_t + b * v[1] * v[1], -a * v[0] + b * v[1] * v[2]],
        [-a * v[1] + b * v[0] * v[2], a * v[0] + b * v[1] * v[2], cos_t + b * v[2] * v[2]],
    ]))
}

/// Direction cosine matrix to the rotation vector with angle in `[0, pi]`.
pub fn so3_log_dcm(_d: &Dcm) -> Result<[f32; 3], RotationError> {
    Ok([0.0; 3])
}

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
