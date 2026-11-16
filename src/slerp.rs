//! Quaternion spherical linear and normalized linear interpolation.

use crate::math::{MathError, Quaternion};

#[inline]
pub fn quat_dot(a: &Quaternion, b: &Quaternion) -> f32 {
    a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
}

/// Normalized linear interpolation along the shortest geodesic path.
pub fn quat_nlerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    if !t.is_finite() {
        return Err(MathError::InvalidNorm);
    }
    let na = a.normalize()?;
    let nb = b.normalize()?;
    let s = if quat_dot(&na, &nb) < 0.0 { -1.0 } else { 1.0 };
    Quaternion::new(
        na.w + t * (s * nb.w - na.w),
        na.x + t * (s * nb.x - na.x),
        na.y + t * (s * nb.y - na.y),
        na.z + t * (s * nb.z - na.z),
    )
    .normalize()
}

#[inline]
fn slerp_weights(cos_theta: f32, sign: f32, t: f32) -> (f32, f32) {
    let theta = libm::acosf(cos_theta);
    let inv_sin = 1.0 / libm::sinf(theta);
    (
        libm::sinf((1.0 - t) * theta) * inv_sin,
        sign * libm::sinf(t * theta) * inv_sin,
    )
}

/// Spherical linear interpolation between two quaternions along the shortest path.
pub fn quat_slerp(a: &Quaternion, b: &Quaternion, t: f32) -> Result<Quaternion, MathError> {
    if !t.is_finite() {
        return Err(MathError::InvalidNorm);
    }
    let na = a.normalize()?;
    let nb = b.normalize()?;
    if t <= 0.0 {
        return Ok(na);
    }
    if t >= 1.0 {
        return Ok(nb);
    }
    let d = quat_dot(&na, &nb);
    let sign = if d < 0.0 { -1.0 } else { 1.0 };
    let cos_t = (d * sign).clamp(-1.0, 1.0);
    if cos_t > 0.9995 {
        return quat_nlerp(&na, &nb, t);
    }
    let (wa, wb) = slerp_weights(cos_t, sign, t);
    Quaternion::new(
        wa * na.w + wb * nb.w,
        wa * na.x + wb * nb.x,
        wa * na.y + wb * nb.y,
        wa * na.z + wb * nb.z,
    )
    .normalize()
}

pub use quat_nlerp as nlerp;
pub use quat_slerp as slerp;
