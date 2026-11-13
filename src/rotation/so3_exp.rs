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
