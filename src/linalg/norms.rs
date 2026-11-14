//! Matrix norms, trace, and finite checks.

use crate::math::Mat;

/// Returns the Frobenius norm of matrix `a`: `sqrt(sum(a_ij^2))`.
pub fn frobenius_norm<const R: usize, const C: usize>(a: &Mat<R, C>) -> f32 {
    let mut sum = 0.0;
    for r in 0..R {
        for c in 0..C {
            let v = a.0[r][c];
            sum += v * v;
        }
    }
    libm::sqrtf(sum)
}

/// Returns the maximum absolute entry of matrix `a`: `max(|a_ij|)`.
pub fn max_abs_norm<const R: usize, const C: usize>(a: &Mat<R, C>) -> f32 {
    let mut max_val = 0.0;
    for r in 0..R {
        for c in 0..C {
            let abs_v = libm::fabsf(a.0[r][c]);
            if abs_v > max_val {
                max_val = abs_v;
            }
        }
    }
    max_val
}

/// Returns the trace of square matrix `a`: `sum(a_ii)`.
pub fn trace<const N: usize>(a: &Mat<N, N>) -> f32 {
    let mut tr = 0.0;
    for i in 0..N {
        tr += a.0[i][i];
    }
    tr
}

/// Returns true if all entries in matrix `a` are finite.
pub fn all_finite<const R: usize, const C: usize>(a: &Mat<R, C>) -> bool {
    for r in 0..R {
        for c in 0..C {
            if !a.0[r][c].is_finite() {
                return false;
            }
        }
    }
    true
}
