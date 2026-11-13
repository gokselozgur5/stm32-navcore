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
