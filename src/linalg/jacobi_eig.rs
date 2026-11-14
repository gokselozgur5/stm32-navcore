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

/// Computes a single cyclic Jacobi sweep over all pairs `(p, q)`.
#[allow(clippy::needless_range_loop)]
fn cyclic_jacobi_sweep<const N: usize>(a: &mut Mat<N, N>, v: &mut Mat<N, N>) {
    for p in 0..N {
        for q in (p + 1)..N {
            let (c, s) = jacobi_rotation(a.0[p][p], a.0[q][q], a.0[p][q]);
            let app = c * c * a.0[p][p] - 2.0 * s * c * a.0[p][q] + s * s * a.0[q][q];
            let aqq = s * s * a.0[p][p] + 2.0 * s * c * a.0[p][q] + c * c * a.0[q][q];
            apply_jacobi_step(a, v, p, q, c, s);
            a.0[p][p] = app;
            a.0[q][q] = aqq;
            a.0[p][q] = 0.0;
            a.0[q][p] = 0.0;
        }
    }
}

/// Maximum number of cyclic sweeps permitted.
const MAX_SWEEPS: usize = 50;

/// Tolerance for off-diagonal Frobenius norm squared.
const EPS_SQ: f32 = 1e-12;

/// Checks that all entries in matrix are finite.
#[inline]
fn validate_finite<const N: usize>(a: &Mat<N, N>) -> Result<(), LinalgError> {
    if a.0.iter().flatten().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err(LinalgError::NonFinite)
    }
}

/// Computes eigenvalues and eigenvectors of real symmetric matrix `a`.
#[allow(clippy::needless_range_loop)]
pub fn jacobi_eig<const N: usize>(a: &Mat<N, N>) -> Result<Eigen<N>, LinalgError> {
    validate_finite(a)?;
    let mut d = *a;
    let mut v = Mat::<N, N>::identity();
    for _ in 0..MAX_SWEEPS {
        if off_diagonal_norm_sq(&d) <= EPS_SQ {
            break;
        }
        cyclic_jacobi_sweep(&mut d, &mut v);
    }
    let mut values = [0.0f32; N];
    for i in 0..N {
        values[i] = d.0[i][i];
    }
    Ok(Eigen { values, vectors: v })
}
