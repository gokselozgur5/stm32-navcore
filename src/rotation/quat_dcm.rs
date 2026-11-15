//! Conversion between unit quaternions and direction cosine matrices.

use super::{Dcm, RotationError};
use crate::math::{Mat, MathError, Quaternion};

impl Dcm {
    /// Builds the matrix that rotates a vector like `q.rotate_vec`.
    pub fn from_quaternion(q: &Quaternion) -> Result<Self, MathError> {
        let Quaternion { w, x, y, z } = q.normalize()?;
        Ok(Self(Mat([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ])))
    }

    /// Builds the matrix directly from a pre-normalized unit quaternion.
    pub fn from_quaternion_normalized(q: &Quaternion) -> Self {
        let Quaternion { w, x, y, z } = *q;
        Self(Mat([
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]))
    }
}

impl Dcm {
    /// Largest-diagonal branch of the Shepperd method.
    fn quat_from_diagonal(&self) -> Quaternion {
        let r = &self.0 .0;
        let mut k = 0;
        for d in 1..3 {
            if r[d][d] > r[k][k] {
                k = d;
            }
        }
        let (i, j) = ((k + 1) % 3, (k + 2) % 3);
        let s = 2.0 * libm::sqrtf(1.0 + r[k][k] - r[i][i] - r[j][j]);
        let mut q = [0.0f32; 4];
        q[0] = (r[j][i] - r[i][j]) / s;
        q[k + 1] = 0.25 * s;
        q[i + 1] = (r[i][k] + r[k][i]) / s;
        q[j + 1] = (r[j][k] + r[k][j]) / s;
        Quaternion::new(q[0], q[1], q[2], q[3])
    }
}

impl Dcm {
    /// Converts to a unit quaternion with the Shepperd method.
    pub fn to_quaternion(&self) -> Quaternion {
        let r = &self.0 .0;
        let tr = r[0][0] + r[1][1] + r[2][2];
        if tr > r[0][0].max(r[1][1]).max(r[2][2]) {
            let s = 2.0 * libm::sqrtf(1.0 + tr);
            return Quaternion::new(
                0.25 * s,
                (r[2][1] - r[1][2]) / s,
                (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s,
            );
        }
        self.quat_from_diagonal()
    }

    /// Converts to a unit quaternion after validating orthonormality.
    pub fn try_to_quaternion(&self) -> Result<Quaternion, RotationError> {
        let err = self.orthogonality_error();
        let det_err = (self.det() - 1.0).abs();
        if !(err <= super::dcm::DCM_TOLERANCE && det_err <= super::dcm::DCM_TOLERANCE) {
            return Err(RotationError::NotOrthonormal);
        }
        Ok(self.to_quaternion())
    }
}

impl Quaternion {
    /// Converts a unit quaternion to a direction cosine matrix.
    pub fn to_dcm(&self) -> Result<Dcm, MathError> {
        Dcm::from_quaternion(self)
    }

    /// Converts a direction cosine matrix to a unit quaternion.
    pub fn from_dcm(dcm: &Dcm) -> Self {
        dcm.to_quaternion()
    }

    /// Converts a direction cosine matrix to a unit quaternion with validation.
    pub fn try_from_dcm(dcm: &Dcm) -> Result<Self, RotationError> {
        dcm.try_to_quaternion()
    }
}
