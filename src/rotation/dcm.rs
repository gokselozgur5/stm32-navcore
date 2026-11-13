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
        for (i, row) in p.0.iter().enumerate() {
            for (j, &val) in row.iter().enumerate() {
                let target = if i == j { 1.0 } else { 0.0 };
                err = err.max((val - target).abs());
            }
        }
        err
    }
}

impl Dcm {
    /// Determinant of the matrix.
    pub fn det(&self) -> f32 {
        let m = &self.0 .0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Validates `m` as a proper rotation within [`DCM_TOLERANCE`].
    pub fn try_new(m: Mat<3, 3>) -> Result<Self, RotationError> {
        let d = Self(m);
        let err = d.orthogonality_error();
        let det_err = (d.det() - 1.0).abs();
        if !(err <= DCM_TOLERANCE && det_err <= DCM_TOLERANCE) {
            return Err(RotationError::NotOrthonormal);
        }
        Ok(d)
    }
}

impl Dcm {
    /// Composition `self * rhs`.
    pub fn mul(&self, rhs: &Self) -> Self {
        Self(self.0.matmul(&rhs.0))
    }

    /// Inverse rotation.
    pub fn transpose(&self) -> Self {
        Self(self.0.transpose())
    }
}
