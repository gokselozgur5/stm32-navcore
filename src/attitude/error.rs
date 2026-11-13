//! Attitude error angle and rotation axis between two quaternions.

use crate::math::{MathError, Quaternion};

/// Relative attitude error between two quaternions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelativeAttitude {
    /// Error rotation angle in radians, in the range `[0, PI]`.
    pub angle_rad: f32,
    /// Unit rotation axis `[ux, uy, uz]`. When angle is near zero, defaults to `[0.0, 0.0, 0.0]`.
    pub axis: [f32; 3],
}

/// Compute the attitude error angle and axis from `from` to `to`.
pub fn attitude_error(from: &Quaternion, to: &Quaternion) -> Result<RelativeAttitude, MathError> {
    let q_from = from.normalize()?;
    let q_to = to.normalize()?;

    let q_from_conj = Quaternion::new(q_from.w, -q_from.x, -q_from.y, -q_from.z);
    let mut dq = q_from_conj.mul(&q_to);

    if dq.w < 0.0 {
        dq.w = -dq.w;
        dq.x = -dq.x;
        dq.y = -dq.y;
        dq.z = -dq.z;
    }

    let v_norm = libm::sqrtf(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
    let angle_rad = 2.0 * libm::atan2f(v_norm, dq.w);

    let axis = if v_norm > 1e-7 {
        let inv = 1.0 / v_norm;
        [dq.x * inv, dq.y * inv, dq.z * inv]
    } else {
        [0.0, 0.0, 0.0]
    };

    Ok(RelativeAttitude { angle_rad, axis })
}
