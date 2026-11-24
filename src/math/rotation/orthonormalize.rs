//! Skew matrix and re-orthonormalization of drifting rotation matrices.

use super::{Dcm, RotationError};
use crate::math::Mat;

/// Cross-product matrix: `skew(a) * b == a x b`.
pub fn skew(v: [f32; 3]) -> Mat<3, 3> {
    Mat([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
}

/// Extracts the 3D vector from a skew-symmetric matrix.
pub fn unskew(m: &Mat<3, 3>) -> [f32; 3] {
    [
        0.5 * (m.0[2][1] - m.0[1][2]),
        0.5 * (m.0[0][2] - m.0[2][0]),
        0.5 * (m.0[1][0] - m.0[0][1]),
    ]
}

#[inline]
fn vec3_norm(v: [f32; 3]) -> f32 {
    let s = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if s > 0.0 {
        libm::sqrtf(s)
    } else {
        0.0
    }
}

impl Dcm {
    /// Restores orthogonality with the iteration `R <- R (1.5 I - 0.5 R^T R)`.
    ///
    /// Converges quadratically for matrices near a rotation.
    pub fn orthonormalize(&self) -> Result<Dcm, RotationError> {
        let mut r = self.0;
        for _ in 0..4 {
            let rtr = r.transpose().matmul(&r);
            let mut corr = Mat::<3, 3>::zeros();
            for i in 0..3 {
                for j in 0..3 {
                    let id = if i == j { 1.5 } else { 0.0 };
                    corr.0[i][j] = id - 0.5 * rtr.0[i][j];
                }
            }
            r = r.matmul(&corr);
        }
        if r.0.iter().flatten().any(|x| !x.is_finite()) {
            return Err(RotationError::NotOrthonormal);
        }
        Dcm::try_new(r)
    }

    /// Restores orthogonality via Schulz iterative polar re-orthonormalization.
    #[inline]
    pub fn orthonormalize_polar(&self) -> Result<Dcm, RotationError> {
        self.orthonormalize()
    }

    /// Restores orthogonality via Gram-Schmidt projection and cross-product.
    pub fn orthonormalize_gram_schmidt(&self) -> Result<Dcm, RotationError> {
        if self.det() <= 0.0 {
            return Err(RotationError::NotOrthonormal);
        }
        let (r0, r1) = (self.0 .0[0], self.0 .0[1]);
        let nx = vec3_norm(r0);
        if nx < 1e-6 {
            return Err(RotationError::NotOrthonormal);
        }
        let x = [r0[0] / nx, r0[1] / nx, r0[2] / nx];
        let p = r1[0] * x[0] + r1[1] * x[1] + r1[2] * x[2];
        let y_raw = [r1[0] - p * x[0], r1[1] - p * x[1], r1[2] - p * x[2]];
        let ny = vec3_norm(y_raw);
        if ny < 1e-6 {
            return Err(RotationError::NotOrthonormal);
        }
        let y = [y_raw[0] / ny, y_raw[1] / ny, y_raw[2] / ny];
        let z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ];
        let nz = vec3_norm(z);
        if nz < 1e-6 {
            return Err(RotationError::NotOrthonormal);
        }
        let r = Mat([x, y, [z[0] / nz, z[1] / nz, z[2] / nz]]);
        Dcm::try_new(r)
    }
}

impl Dcm {
    /// Restores orthogonality via symmetric error correction and cross product.
    pub fn orthonormalize_symmetric(&self) -> Result<Dcm, RotationError> {
        let (mut x, mut y) = (self.0 .0[0], self.0 .0[1]);
        let half_err = 0.5 * (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
        for i in 0..3 {
            let (xi, yi) = (x[i] - half_err * y[i], y[i] - half_err * x[i]);
            x[i] = xi;
            y[i] = yi;
        }
        let z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ];
        let nx = vec3_norm(x);
        let ny = vec3_norm(y);
        let nz = vec3_norm(z);
        if nx <= 1e-6
            || !nx.is_finite()
            || ny <= 1e-6
            || !ny.is_finite()
            || nz <= 1e-6
            || !nz.is_finite()
        {
            return Err(RotationError::NotOrthonormal);
        }
        let r0 = [x[0] / nx, x[1] / nx, x[2] / nx];
        let r1 = [y[0] / ny, y[1] / ny, y[2] / ny];
        let r2 = [z[0] / nz, z[1] / nz, z[2] / nz];
        Dcm::try_new(Mat([r0, r1, r2]))
    }
}
