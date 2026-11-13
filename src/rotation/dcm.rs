//! Direction cosine matrix with an orthonormality check.

use crate::math::Mat;

/// Errors reported by rotation utilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotationError {
    /// The matrix is not a proper rotation or holds non-finite values.
    NotOrthonormal,
}

/// A 3x3 rotation matrix. The field is public, so unchecked values are
/// possible; use [`Dcm::try_new`] to validate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dcm(pub Mat<3, 3>);

impl Dcm {
    /// The identity rotation.
    pub fn identity() -> Self {
        Self(Mat::identity())
    }
}

/// Largest accepted orthogonality or determinant error.
pub const DCM_TOLERANCE: f32 = 1e-4;

impl Dcm {
    /// Largest absolute entry of `R * R^T - I`.
    pub fn orthogonality_error(&self) -> f32 {
        let p = self.0.matmul(&self.0.transpose());
        let mut err = 0.0f32;
        for i in 0..3 {
            for j in 0..3 {
                let target = if i == j { 1.0 } else { 0.0 };
                err = err.max((p.0[i][j] - target).abs());
            }
        }
        err
    }
}
