//! Attitude error between two quaternions.

/// Smallest rotation taking one attitude to another.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeError {
    /// Rotation angle in `[0, pi]` radians.
    pub angle: f32,
    /// Unit rotation axis, or zero when the angle is zero.
    pub axis: [f32; 3],
}
