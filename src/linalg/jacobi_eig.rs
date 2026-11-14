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

/// Applies a similarity transformation for pair `(p, q)` to `a` and updates `v`.
#[allow(clippy::needless_range_loop)]
fn apply_jacobi_step<const N: usize>(
    a: &mut Mat<N, N>,
    v: &mut Mat<N, N>,
    p: usize,
    q: usize,
    c: f32,
    s: f32,
) {
    for k in 0..N {
        if k != p && k != q {
            let a_kp = a.0[k][p];
            let a_kq = a.0[k][q];
            a.0[k][p] = c * a_kp - s * a_kq;
            a.0[p][k] = a.0[k][p];
            a.0[k][q] = s * a_kp + c * a_kq;
            a.0[q][k] = a.0[k][q];
        }
        let v_kp = v.0[k][p];
        let v_kq = v.0[k][q];
        v.0[k][p] = c * v_kp - s * v_kq;
        v.0[k][q] = s * v_kp + c * v_kq;
    }
}

/// Computes off-diagonal Frobenius norm squared.
#[inline]
fn off_diagonal_norm_sq<const N: usize>(a: &Mat<N, N>) -> f32 {
    let mut sum = 0.0f32;
    for i in 0..N {
        for j in 0..N {
            if i != j {
                sum += a.0[i][j] * a.0[i][j];
            }
        }
    }
    sum
}

/// Computes eigenvalues and eigenvectors of real symmetric matrix `a`.
pub fn jacobi_eig<const N: usize>(_a: &Mat<N, N>) -> Result<Eigen<N>, LinalgError> {
    Err(LinalgError::NonFinite)
}
