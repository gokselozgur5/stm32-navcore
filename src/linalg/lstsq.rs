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
