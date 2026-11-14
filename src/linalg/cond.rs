//! One-norm condition number estimation for symmetric positive definite matrices.

use crate::math::Mat;

/// Computes the 1-norm (maximum absolute column sum) of matrix `a`.
pub fn norm_1<const R: usize, const C: usize>(a: &Mat<R, C>) -> f32 {
    let mut max_col = 0.0f32;
    for c in 0..C {
        let mut col_sum = 0.0f32;
        for r in 0..R {
            col_sum += libm::fabsf(a.0[r][c]);
        }
        if col_sum > max_col {
            max_col = col_sum;
        }
    }
    max_col
}

use super::cholesky::cholesky;
use super::triangular::{solve_lower, solve_upper};
use super::LinalgError;

fn solve_spd_l<const N: usize>(l: &Mat<N, N>, b: &[f32; N]) -> Result<[f32; N], LinalgError> {
    let y = solve_lower(l, b)?;
    solve_upper(&l.transpose(), &y)
}
