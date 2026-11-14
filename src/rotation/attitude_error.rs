//! Attitude error between two quaternions.

/// Smallest rotation taking one attitude to another.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeError {
    /// Rotation angle in `[0, pi]` radians.
    pub angle: f32,
    /// Unit rotation axis, or zero when the angle is zero.
    pub axis: [f32; 3],
}
use crate::math::{MathError, Quaternion};

/// Error `conj(a) * b` expressed as an angle and an axis.
pub fn attitude_error(a: &Quaternion, b: &Quaternion) -> Result<AttitudeError, MathError> {
    let ca = Quaternion::new(a.w, -a.x, -a.y, -a.z);
    let mut d = ca.mul(b).normalize()?;
    if d.w < 0.0 {
        d = Quaternion::new(-d.w, -d.x, -d.y, -d.z);
    }
    let vee = libm::sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
    let angle = 2.0 * libm::atan2f(vee, d.w);
    let axis = if vee < 1e-9 {
        [0.0; 3]
    } else {
        [d.x / vee, d.y / vee, d.z / vee]
    };
    Ok(AttitudeError { angle, axis })
}
