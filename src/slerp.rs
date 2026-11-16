//! Quaternion spherical linear and normalized linear interpolation.

use crate::math::{MathError, Quaternion};

#[inline]
pub(crate) fn quat_dot(a: &Quaternion, b: &Quaternion) -> f32 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}
