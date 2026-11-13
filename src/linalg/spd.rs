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
