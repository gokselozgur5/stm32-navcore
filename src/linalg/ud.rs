//! UD factorization with a unit upper factor and a diagonal.

use super::LinalgError;
use crate::math::Mat;

/// Returns `(U, D)` with `U D U^T = P`, where `U` has a unit diagonal.
#[allow(clippy::needless_range_loop)]
pub fn ud_factor<const N: usize>(p: &Mat<N, N>) -> Result<(Mat<N, N>, [f32; N]), LinalgError> {
    if !p.0.iter().flatten().all(|v| v.is_finite()) {
        return Err(LinalgError::NonFinite);
    }
    let mut u = Mat::<N, N>::identity();
    let mut d = [0.0f32; N];
    for j in (0..N).rev() {
        for i in (0..=j).rev() {
            let mut s = p.0[i][j];
            for k in j + 1..N {
                s -= u.0[i][k] * u.0[j][k] * d[k];
            }
            if i < j {
                u.0[i][j] = s / d[j];
            } else if s > 0.0 {
                d[j] = s;
            } else {
                return Err(LinalgError::NotPositiveDefinite);
            }
        }
    }
    Ok((u, d))
}
