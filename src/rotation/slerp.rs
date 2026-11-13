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

/// Spherical linear interpolation along the shortest path.
pub fn slerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    let d = dot(a, b);
    let s = if d < 0.0 { -1.0 } else { 1.0 };
    let theta = libm::acosf(d * s);
    let k = 1.0 / libm::sinf(theta);
    let (wa, wb) = (
        libm::sinf((1.0 - t) * theta) * k,
        s * libm::sinf(t * theta) * k,
    );
    Quaternion::new(
        wa * a.w + wb * b.w,
        wa * a.x + wb * b.x,
        wa * a.y + wb * b.y,
        wa * a.z + wb * b.z,
    )
    .normalize()
}
