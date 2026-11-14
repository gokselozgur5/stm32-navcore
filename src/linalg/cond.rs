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

fn argmax_abs<const N: usize>(v: &[f32; N]) -> usize {
    let mut best_i = 0;
    let mut best_val = 0.0f32;
    for (i, &val) in v.iter().enumerate() {
        let a = libm::fabsf(val);
        if a > best_val {
            best_val = a;
            best_i = i;
        }
    }
    best_i
}

fn step_est<const N: usize>(l: &Mat<N, N>, x: &mut [f32; N]) -> Result<f32, LinalgError> {
    let y = solve_spd_l(l, x)?;
    let mut gamma = 0.0f32;
    let mut xi = [0.0f32; N];
    for i in 0..N {
        gamma += libm::fabsf(y[i]);
        xi[i] = if y[i] >= 0.0 { 1.0 } else { -1.0 };
    }
    let z = solve_spd_l(l, &xi)?;
    let mut dot = 0.0f32;
    for i in 0..N {
        dot += z[i] * x[i];
    }
    let k = argmax_abs(&z);
    if libm::fabsf(z[k]) > dot {
        *x = [0.0f32; N];
        x[k] = 1.0;
    }
    Ok(gamma)
}

/// Estimates condition number $\kappa_1(A) = \|A\|_1 \|A^{-1}\|_1$ for SPD matrix `a`.
pub fn cond_est_spd<const N: usize>(a: &Mat<N, N>) -> Result<f32, LinalgError> {
    if N == 0 {
        return Ok(1.0);
    }
    let norm_a = norm_1(a);
    let l = cholesky(a)?;
    let mut x = [1.0f32 / (N as f32); N];
    let mut gamma = 0.0f32;
    for _ in 0..5 {
        gamma = step_est(&l, &mut x)?;
    }
    Ok(norm_a * gamma)
}

/// Estimates reciprocal 1-norm condition number $1 / \kappa_1(A)$ for SPD matrix `a`.
pub fn rcond_est_spd<const N: usize>(a: &Mat<N, N>) -> Result<f32, LinalgError> {
    let cond = cond_est_spd(a)?;
    if cond <= 0.0 || !cond.is_finite() {
        Ok(0.0)
    } else {
        Ok(1.0 / cond)
    }
}

/// Returns true if matrix `a` is SPD and its condition number estimate <= `max_cond`.
pub fn is_well_conditioned_spd<const N: usize>(a: &Mat<N, N>, max_cond: f32) -> bool {
    match cond_est_spd(a) {
        Ok(cond) => cond <= max_cond && cond.is_finite(),
        Err(_) => false,
    }
}
