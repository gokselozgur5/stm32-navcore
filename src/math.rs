//! Math module: Matrix algebra, Quaternion, and tilt-compensated heading.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MathError {
    SingularMatrix,
    InvalidNorm,
    GimbalLock,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat<const R: usize, const C: usize>(pub [[f32; C]; R]);

impl<const R: usize, const C: usize> Mat<R, C> {
    pub const fn zeros() -> Self {
        Self([[0.0; C]; R])
    }

    pub fn identity() -> Self {
        let mut out = [[0.0; C]; R];
        let n = if R < C { R } else { C };
        let mut i = 0;
        while i < n {
            out[i][i] = 1.0;
            i += 1;
        }
        Self(out)
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn add(&self, other: &Self) -> Self {
        let mut out = [[0.0; C]; R];
        for r in 0..R {
            for c in 0..C {
                out[r][c] = self.0[r][c] + other.0[r][c];
            }
        }
        Self(out)
    }

    #[allow(clippy::needless_range_loop)]
    pub fn sub(&self, other: &Self) -> Self {
        let mut out = [[0.0; C]; R];
        for r in 0..R {
            for c in 0..C {
                out[r][c] = self.0[r][c] - other.0[r][c];
            }
        }
        Self(out)
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn matmul<const K: usize>(&self, other: &Mat<C, K>) -> Mat<R, K> {
        let mut out = [[0.0; K]; R];
        for r in 0..R {
            for k in 0..K {
                let mut sum = 0.0f32;
                for c in 0..C {
                    sum += self.0[r][c] * other.0[c][k];
                }
                out[r][k] = sum;
            }
        }
        Mat(out)
    }
}
