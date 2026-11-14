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
