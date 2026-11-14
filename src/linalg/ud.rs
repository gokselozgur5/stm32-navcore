use core::ops::{AddAssign, DivAssign};

/// Unit upper U and diagonal D for covariance matrices.
#[derive(Debug)]
pub struct Ud {
    pub u: Vec<f32>,
    pub d: Vec<f32>,
}

impl Ud {
    /// Creates a new `Ud` from a 8x8 covariance matrix.
    pub fn new(cov: &[[f32; 8]; 8]) -> Self {
        let mut u = vec![0.0; 8];
        let mut d = vec![1.0; 8];

        for i in 0..8 {
            for j in (i + 1)..8 {
                if cov[i][j] != 0.0 {
                    let norm = cov[j][j].sqrt();
                    u[i] += cov[i][j] / norm;
                    d[j] /= norm;
                }
            }
        }
