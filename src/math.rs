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

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn transpose(&self) -> Mat<C, R> {
        let mut out = [[0.0; R]; C];
        for r in 0..R {
            for c in 0..C {
                out[c][r] = self.0[r][c];
            }
        }
        Mat(out)
    }
}

impl<const N: usize> Mat<N, N> {
    #[allow(clippy::needless_range_loop)]
    pub fn symmetrize(&self) -> Self {
        let mut out = [[0.0; N]; N];
        for r in 0..N {
            for c in 0..N {
                out[r][c] = 0.5 * (self.0[r][c] + self.0[c][r]);
            }
        }
        Mat(out)
    }
}

impl Mat<1, 1> {
    pub fn inv_1x1(&self) -> Option<Self> {
        let v = self.0[0][0];
        let abs_v = if v < 0.0 { -v } else { v };
        if abs_v < 1e-12 {
            None
        } else {
            Some(Mat([[1.0 / v]]))
        }
    }
}
