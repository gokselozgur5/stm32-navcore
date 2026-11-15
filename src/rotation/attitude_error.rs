//! Attitude error between two quaternions.

use crate::math::{MathError, Quaternion};

/// Smallest rotation taking one attitude to another.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeError {
    /// Rotation angle in `[0, pi]` radians.
    pub angle: f32,
    /// Unit rotation axis, or zero when the angle is zero.
    pub axis: [f32; 3],
}

impl AttitudeError {
    /// Zero rotation error.
    pub const IDENTITY: Self = Self {
        angle: 0.0,
        axis: [0.0; 3],
    };

    /// Rotation vector representation `axis * angle`.
    pub fn rotation_vector(&self) -> [f32; 3] {
        [
            self.axis[0] * self.angle,
            self.axis[1] * self.angle,
            self.axis[2] * self.angle,
        ]
    }

    /// Rotation error angle converted to degrees.
    pub fn angle_deg(&self) -> f32 {
        self.angle * (180.0 / core::f32::consts::PI)
    }
}

/// Error `conj(a) * b` expressed as an angle and an axis.
pub fn attitude_error(a: &Quaternion, b: &Quaternion) -> Result<AttitudeError, MathError> {
    let qa = a.normalize()?;
    let qb = b.normalize()?;
    let ca = Quaternion::new(qa.w, -qa.x, -qa.y, -qa.z);
    let mut d = ca.mul(&qb).normalize()?;
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

/// Relative attitude error expressed directly as a rotation vector `axis * angle`.
pub fn attitude_error_vector(a: &Quaternion, b: &Quaternion) -> Result<[f32; 3], MathError> {
    attitude_error(a, b).map(|err| err.rotation_vector())
}
