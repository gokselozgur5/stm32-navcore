//! Overdetermined least-squares solve through Householder QR.

use super::LinalgError;
use crate::math::Mat;

/// Returns the Householder vector `v` and `v^T v` that zero column `k` below the diagonal.
pub fn reflector<const M: usize, const N: usize>(
    r: &[[f32; N]; M],
    k: usize,
) -> Result<([f32; M], f32), LinalgError> {
    let norm = libm::sqrtf((k..M).map(|i| r[i][k] * r[i][k]).sum::<f32>());
    if norm == 0.0 || !norm.is_finite() {
        return Err(LinalgError::ZeroPivot);
    }
    let alpha = if r[k][k] > 0.0 { -norm } else { norm };
    let mut v = [0.0f32; M];
    for i in k..M {
        v[i] = r[i][k];
    }
    v[k] -= alpha;
    Ok((v, v.iter().map(|x| x * x).sum()))
}

/// Applies `I - 2 v v^T / v^T v` to rows `k..M` of one column.
pub fn reflect_col<const M: usize>(v: &[f32; M], vtv: f32, k: usize, c: &mut [f32; M]) {
    let f = 2.0 * (k..M).map(|i| v[i] * c[i]).sum::<f32>() / vtv;
    for i in k..M {
        c[i] -= f * v[i];
    }
}

/// Eliminates column `k` of `r` and applies the same reflection to `y`.
pub fn householder_step<const M: usize, const N: usize>(
    r: &mut [[f32; N]; M],
    y: &mut [f32; M],
    k: usize,
) -> Result<(), LinalgError> {
    let (v, vtv) = reflector(r, k)?;
    for j in k..N {
        let mut col = [0.0f32; M];
        for i in 0..M {
            col[i] = r[i][j];
        }
        reflect_col(&v, vtv, k, &mut col);
        for i in 0..M {
            r[i][j] = col[i];
        }
    }
    reflect_col(&v, vtv, k, y);
    Ok(())
}

/// Solves `R x = y` for the leading `N` rows of an upper triangular `R`.
pub fn back_substitute<const M: usize, const N: usize>(
    r: &[[f32; N]; M],
    y: &[f32; M],
) -> Result<[f32; N], LinalgError> {
    let mut x = [0.0f32; N];
    for k in (0..N).rev() {
        if r[k][k] == 0.0 {
            return Err(LinalgError::ZeroPivot);
        }
        let s: f32 = (k + 1..N).map(|j| r[k][j] * x[j]).sum();
        x[k] = (y[k] - s) / r[k][k];
    }
    Ok(x)
}

/// Minimizes `|A x - b|` for a full column rank `A` with `M >= N`.
///
/// Fails with `ZeroPivot` when `M < N` or `A` is rank deficient.
pub fn lstsq<const M: usize, const N: usize>(
    a: &Mat<M, N>,
    b: &[f32; M],
) -> Result<[f32; N], LinalgError> {
    if M < N {
        return Err(LinalgError::ZeroPivot);
    }
    if !a.0.iter().flatten().chain(b.iter()).all(|v| v.is_finite()) {
        return Err(LinalgError::NonFinite);
    }
    let (mut r, mut y) = (a.0, *b);
    for k in 0..N {
        householder_step(&mut r, &mut y, k)?;
    }
    back_substitute(&r, &y)
}
