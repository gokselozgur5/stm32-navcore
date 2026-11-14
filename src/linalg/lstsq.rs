//! Overdetermined least-squares solve through Householder QR.

use super::LinalgError;
use crate::math::Mat;

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

#[allow(clippy::needless_range_loop)]
fn project_qt_b<const M: usize, const N: usize>(q: &Mat<M, M>, b: &[f32; M]) -> [f32; N] {
    let mut c = [0.0f32; N];
    for j in 0..N {
        for i in 0..M {
            c[j] += q.0[i][j] * b[i];
        }
    }
    c
}

#[allow(clippy::needless_range_loop)]
fn solve_lstsq_qr<const M: usize, const N: usize>(
    q: &Mat<M, M>,
    r: &Mat<M, N>,
    b: &[f32; M],
) -> Result<[f32; N], LinalgError> {
    let c: [f32; N] = project_qt_b(q, b);
    let mut x = [0.0f32; N];
    for i in (0..N).rev() {
        let mut s = c[i];
        for k in i + 1..N {
            s -= r.0[i][k] * x[k];
        }
        let diag = r.0[i][i];
        if !diag.is_finite() || diag.abs() < 1e-6 {
            return Err(LinalgError::ZeroPivot);
        }
        x[i] = s / diag;
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
    if !b.iter().all(|v| v.is_finite()) {
        return Err(LinalgError::NonFinite);
    }
    let (q, r) = super::qr::qr(a)?;
    solve_lstsq_qr(&q, &r, b)
}
