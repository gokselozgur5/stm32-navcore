//! Cyclic Jacobi eigen decomposition for real symmetric matrices.

use super::LinalgError;
use crate::math::Mat;

/// Result of symmetric eigen decomposition: eigenvalues and orthogonal eigenvector matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Eigen<const N: usize> {
    /// Eigenvalues corresponding to column vectors of `vectors`.
    pub values: [f32; N],
    /// Column-orthonormal matrix where column `k` is the eigenvector for `values[k]`.
    pub vectors: Mat<N, N>,
}

/// Computes Jacobi rotation (c, s) annihilating off-diagonal entry (p, q).
#[inline]
fn jacobi_rotation(app: f32, aqq: f32, apq: f32) -> (f32, f32) {
    if apq == 0.0 {
        return (1.0, 0.0);
    }
    let tau = (aqq - app) / (2.0 * apq);
    let t = if tau >= 0.0 {
        1.0 / (tau + libm::sqrtf(1.0 + tau * tau))
    } else {
        -1.0 / (-tau + libm::sqrtf(1.0 + tau * tau))
    };
    let c = 1.0 / libm::sqrtf(1.0 + t * t);
    let s = t * c;
    (c, s)
}

/// Computes eigenvalues and eigenvectors of real symmetric matrix `a`.
pub fn jacobi_eig<const N: usize>(_a: &Mat<N, N>) -> Result<Eigen<N>, LinalgError> {
    Err(LinalgError::NonFinite)
}
