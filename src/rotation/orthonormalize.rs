//! Skew matrix and re-orthonormalization of drifting rotation matrices.

use super::{Dcm, RotationError};
use crate::math::Mat;

/// Cross-product matrix: `skew(a) * b == a x b`.
pub fn skew(v: [f32; 3]) -> Mat<3, 3> {
    Mat([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
}

impl Dcm {
    /// Restores orthogonality with the iteration `R <- R (1.5 I - 0.5 R^T R)`.
    ///
    /// Converges quadratically for matrices near a rotation.
    pub fn orthonormalize(&self) -> Result<Dcm, RotationError> {
        let mut r = self.0;
        for _ in 0..4 {
            let rtr = r.transpose().matmul(&r);
            let mut corr = Mat::<3, 3>::zeros();
            for i in 0..3 {
                for j in 0..3 {
                    let id = if i == j { 1.5 } else { 0.0 };
                    corr.0[i][j] = id - 0.5 * rtr.0[i][j];
                }
            }
            r = r.matmul(&corr);
        }
        if r.0.iter().flatten().any(|x| !x.is_finite()) {
            return Err(RotationError::NotOrthonormal);
        }
        Ok(Dcm(r))
    }
}
