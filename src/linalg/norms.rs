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
