//! Givens rotations and QR decomposition for small matrices.

use super::LinalgError;
use crate::math::Mat;

/// Planar Givens rotation parameters `(c, s)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Givens {
    /// Cosine of the rotation angle.
    pub c: f32,
    /// Sine of the rotation angle.
    pub s: f32,
    /// Annihilated norm `r = sqrt(a^2 + b^2)`.
    pub r: f32,
}

impl Givens {
    /// Computes `(c, s)` such that `[c s; -s c] * [a; b] = [r; 0]`.
    pub fn compute(a: f32, b: f32) -> Self {
        let (c, s, r) = if b == 0.0 {
            (1.0, 0.0, a)
        } else {
            let r = libm::hypotf(a, b);
            (a / r, b / r, r)
        };
        Self { c, s, r }
    }
}

impl Givens {
    /// Applies rotation to rows `j` and `i` of matrix `r`.
    #[allow(clippy::needless_range_loop)]
    pub fn apply_rows<const M: usize, const N: usize>(
        &self,
        r: &mut Mat<M, N>,
        j: usize,
        i: usize,
    ) {
        for k in 0..N {
            let r_j = r.0[j][k];
            let r_i = r.0[i][k];
            r.0[j][k] = self.c * r_j + self.s * r_i;
            r.0[i][k] = -self.s * r_j + self.c * r_i;
        }
    }

    /// Applies transpose rotation to columns `j` and `i` of `q`.
    #[allow(clippy::needless_range_loop)]
    pub fn apply_cols<const M: usize>(&self, q: &mut Mat<M, M>, j: usize, i: usize) {
        for k in 0..M {
            let q_j = q.0[k][j];
            let q_i = q.0[k][i];
            q.0[k][j] = self.c * q_j + self.s * q_i;
            q.0[k][i] = -self.s * q_j + self.c * q_i;
        }
    }
}
