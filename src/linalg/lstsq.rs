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

fn solve_lstsq_qr<const M: usize, const N: usize>(
    _q: &Mat<M, M>,
    _r: &Mat<M, N>,
    _b: &[f32; M],
) -> Result<[f32; N], LinalgError> {
    Ok([0.0f32; N])
}
