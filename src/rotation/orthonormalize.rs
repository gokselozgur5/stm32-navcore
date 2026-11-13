//! Skew matrix and re-orthonormalization of drifting rotation matrices.

use super::{Dcm, RotationError};
use crate::math::Mat;

/// Cross-product matrix: `skew(a) * b == a x b`.
pub fn skew(v: [f32; 3]) -> Mat<3, 3> {
    Mat([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
}
