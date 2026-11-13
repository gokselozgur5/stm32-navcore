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
