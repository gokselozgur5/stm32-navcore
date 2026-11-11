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
