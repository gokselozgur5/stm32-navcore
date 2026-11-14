//! LDL^T factorization with a unit lower factor and a diagonal.

use super::LinalgError;
use crate::math::Mat;

/// Returns `(L, D)` with `L D L^T = A`, where `L` has a unit diagonal.
#[allow(clippy::needless_range_loop)]
pub fn ldl<const N: usize>(a: &Mat<N, N>) -> Result<(Mat<N, N>, [f32; N]), LinalgError> {
    if !a.0.iter().flatten().all(|v| v.is_finite()) {
        return Err(LinalgError::NonFinite);
    }
    let mut l = Mat::<N, N>::identity();
    let mut d = [0.0f32; N];
    for j in 0..N {
        for i in j..N {
            let mut s = a.0[i][j];
            for k in 0..j {
                s -= l.0[i][k] * l.0[j][k] * d[k];
            }
            if i > j {
                l.0[i][j] = s / d[j];
            } else if s > 0.0 {
                d[j] = s;
            } else {
                return Err(LinalgError::NotPositiveDefinite);
            }
        }
    }
    Ok((l, d))
}

/// Rebuilds `L D L^T` from its factors.
#[allow(clippy::needless_range_loop)]
pub fn ldl_reconstruct<const N: usize>(l: &Mat<N, N>, d: &[f32; N]) -> Mat<N, N> {
    let mut ld = *l;
    for r in 0..N {
        for c in 0..N {
            ld.0[r][c] *= d[c];
        }
    }
    ld.matmul(&l.transpose())
}

/// Computes determinant of matrix A from its LDL^T diagonal factor D.
pub fn ldl_det<const N: usize>(d: &[f32; N]) -> f32 {
    let mut prod = 1.0f32;
    for di in d {
        prod *= di;
    }
    prod
}

/// Computes log determinant of matrix A from its LDL^T diagonal factor D.
pub fn ldl_log_det<const N: usize>(d: &[f32; N]) -> f32 {
    let mut sum = 0.0f32;
    for di in d {
        sum += libm::logf(*di);
    }
    sum
}

/// Solves `A x = b` for SPD matrix `A` decomposed into `(L, D)` via `ldl`.
#[allow(clippy::needless_range_loop)]
pub fn ldl_solve<const N: usize>(
    l: &Mat<N, N>,
    d: &[f32; N],
    b: &[f32; N],
) -> Result<[f32; N], LinalgError> {
    for &v in d {
        if !v.is_finite() {
            return Err(LinalgError::NonFinite);
        }
    }
    let y = super::triangular::solve_lower(l, b)?;
    let mut z = [0.0f32; N];
    for i in 0..N {
        if d[i] == 0.0 {
            return Err(LinalgError::ZeroPivot);
        }
        z[i] = y[i] / d[i];
    }
    super::triangular::solve_upper(&l.transpose(), &z)
}
