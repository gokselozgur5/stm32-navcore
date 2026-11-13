//! Forward and back substitution for triangular systems.

use super::LinalgError;
use crate::math::Mat;

/// Solves `L x = b` for lower triangular `L`.
#[allow(clippy::needless_range_loop)]
pub fn solve_lower<const N: usize>(l: &Mat<N, N>, b: &[f32; N]) -> Result<[f32; N], LinalgError> {
    let mut x = [0.0f32; N];
    for i in 0..N {
        let mut s = b[i];
        for k in 0..i {
            s -= l.0[i][k] * x[k];
        }
        if l.0[i][i] == 0.0 {
            return Err(LinalgError::ZeroPivot);
        }
        x[i] = s / l.0[i][i];
    }
    Ok(x)
}
