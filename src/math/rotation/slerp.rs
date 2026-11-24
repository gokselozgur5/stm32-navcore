//! Quaternion interpolation along shortest geodesic path.

use crate::math::{MathError, Quaternion};

pub use dot as quat_dot;
pub use nlerp as quat_nlerp;
pub use slerp as quat_slerp;

/// Compute dot product between two quaternions.
#[inline]
pub fn dot(a: &Quaternion, b: &Quaternion) -> f32 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

/// Normalized linear interpolation along the shortest path.
pub fn nlerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    if !t.is_finite() {
        return Err(MathError::InvalidNorm);
    }
    let a = a.normalize()?;
    let b = b.normalize()?;
    let s = if dot(&a, &b) < 0.0 { -1.0 } else { 1.0 };
    Quaternion::new(
        a.w + t * (s * b.w - a.w),
        a.x + t * (s * b.x - a.x),
        a.y + t * (s * b.y - a.y),
        a.z + t * (s * b.z - a.z),
    )
    .normalize()
}

/// Spherical linear interpolation along the shortest path.
///
/// Clamps `t <= 0.0` to `a` and `t >= 1.0` to `b`.
pub fn slerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    if !t.is_finite() {
        return Err(MathError::InvalidNorm);
    }
    let a = a.normalize()?;
    let b = b.normalize()?;
    if t <= 0.0 {
        return Ok(a);
    }
    if t >= 1.0 {
        return Ok(b);
    }
    let d = dot(&a, &b);
    let s = if d < 0.0 { -1.0 } else { 1.0 };
    let cos_t = (d * s).clamp(-1.0, 1.0);
    if cos_t > 0.9995 {
        return nlerp(&a, &b, t);
    }
    let theta = libm::acosf(cos_t);
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
