//! Conversion between unit quaternions and direction cosine matrices.

use super::Dcm;
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
}
