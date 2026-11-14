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
