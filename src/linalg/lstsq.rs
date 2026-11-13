//! Overdetermined least-squares solve via QR decomposition.

use super::qr::qr;
use super::LinalgError;
use crate::math::Mat;

/// Solves overdetermined least-squares `min_x ||A x - b||_2` for `M >= N`.
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
    let (q, r) = qr(a)?;
    solve_lstsq_qr(&q, &r, b)
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
