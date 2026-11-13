//! Quaternion interpolation.

use crate::math::{MathError, Quaternion};

fn dot(a: &Quaternion, b: &Quaternion) -> f32 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

/// Normalized linear interpolation along the shortest path.
pub fn nlerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    let s = if dot(a, b) < 0.0 { -1.0 } else { 1.0 };
    Quaternion::new(
        a.w + t * (s * b.w - a.w),
        a.x + t * (s * b.x - a.x),
        a.y + t * (s * b.y - a.y),
        a.z + t * (s * b.z - a.z),
    )
    .normalize()
}
