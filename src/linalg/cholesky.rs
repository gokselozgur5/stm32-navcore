//! Cholesky LL^T factorization.

use super::LinalgError;
use crate::math::Mat;

/// Returns the lower factor `L` with `L L^T = A`, reading the lower triangle of `A`.
#[allow(clippy::needless_range_loop)]
pub fn cholesky<const N: usize>(a: &Mat<N, N>) -> Result<Mat<N, N>, LinalgError> {
    if !a.0.iter().flatten().all(|v| v.is_finite()) {
        return Err(LinalgError::NonFinite);
    }
    let mut l = Mat::<N, N>::zeros();
    for i in 0..N {
        for j in 0..=i {
            let mut s = a.0[i][j];
            for k in 0..j {
                s -= l.0[i][k] * l.0[j][k];
            }
            if i == j {
                if s <= 0.0 {
                    return Err(LinalgError::NotPositiveDefinite);
                }
                l.0[i][j] = libm::sqrtf(s);
            } else {
                l.0[i][j] = s / l.0[j][j];
            }
        }
    }
    Ok(l)
}
