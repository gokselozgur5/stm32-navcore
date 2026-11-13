//! Solve and inverse for symmetric positive definite matrices.

use super::cholesky::cholesky;
use super::triangular::{solve_lower, solve_upper};
use super::LinalgError;
use crate::math::Mat;

/// Solves `A x = b` for SPD `A` using `A = L L^T`.
pub fn spd_solve<const N: usize>(a: &Mat<N, N>, b: &[f32; N]) -> Result<[f32; N], LinalgError> {
    let l = cholesky(a)?;
    let y = solve_lower(&l, b)?;
    solve_upper(&l.transpose(), &y)
}

/// Inverts SPD `A` by solving against each unit vector.
#[allow(clippy::needless_range_loop)]
pub fn spd_inverse<const N: usize>(a: &Mat<N, N>) -> Result<Mat<N, N>, LinalgError> {
    let mut inv = Mat::<N, N>::zeros();
    for c in 0..N {
        let mut e = [0.0f32; N];
        e[c] = 1.0;
        let col = spd_solve(a, &e)?;
        for r in 0..N {
            inv.0[r][c] = col[r];
        }
    }
    Ok(inv)
}
