//! Direction cosine matrix with an orthonormality check.

use crate::math::Mat;

/// Errors reported by rotation utilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotationError {
    /// The matrix is not a proper rotation or holds non-finite values.
    NotOrthonormal,
}

/// A 3x3 rotation matrix. The field is public, so unchecked values are
/// possible; use [`Dcm::try_new`] to validate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dcm(pub Mat<3, 3>);

impl Dcm {
    /// The identity rotation.
    pub fn identity() -> Self {
        Self(Mat::identity())
    }

    /// Rotation about the X axis by `angle` radians.
    pub fn from_rot_x(angle: f32) -> Self {
        let (s, c) = (libm::sinf(angle), libm::cosf(angle));
        Self(Mat([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]))
    }

    /// Rotation about the Y axis by `angle` radians.
    pub fn from_rot_y(angle: f32) -> Self {
        let (s, c) = (libm::sinf(angle), libm::cosf(angle));
        Self(Mat([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]))
    }

    /// Rotation about the Z axis by `angle` radians.
    pub fn from_rot_z(angle: f32) -> Self {
        let (s, c) = (libm::sinf(angle), libm::cosf(angle));
        Self(Mat([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]))
    }

    /// Builds a DCM from Tait-Bryan Euler angles in ZYX (yaw, pitch, roll) order.
    pub fn from_euler_zyx(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self::from_rot_z(yaw)
            .mul(&Self::from_rot_y(pitch))
            .mul(&Self::from_rot_x(roll))
    }
}

/// Largest accepted orthogonality or determinant error.
pub const DCM_TOLERANCE: f32 = 1e-4;

impl Dcm {
    /// Largest absolute entry of `R * R^T - I`.
    pub fn orthogonality_error(&self) -> f32 {
        let p = self.0.matmul(&self.0.transpose());
        let mut err = 0.0f32;
        for (i, row) in p.0.iter().enumerate() {
            for (j, &val) in row.iter().enumerate() {
                let target = if i == j { 1.0 } else { 0.0 };
                err = err.max((val - target).abs());
            }
        }
        err
    }
}

impl Dcm {
    /// Determinant of the matrix.
    pub fn det(&self) -> f32 {
        let m = &self.0 .0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Returns true if the matrix satisfies SO(3) orthonormality within [`DCM_TOLERANCE`].
    pub fn is_orthonormal(&self) -> bool {
        self.orthogonality_error() <= DCM_TOLERANCE && (self.det() - 1.0).abs() <= DCM_TOLERANCE
    }

    /// Validates `m` as a proper rotation within [`DCM_TOLERANCE`].
    pub fn try_new(m: Mat<3, 3>) -> Result<Self, RotationError> {
        let d = Self(m);
        if !d.is_orthonormal() {
            return Err(RotationError::NotOrthonormal);
        }
        Ok(d)
    }
}

impl Dcm {
    /// Composition `self * rhs`.
    pub fn mul(&self, rhs: &Self) -> Self {
        Self(self.0.matmul(&rhs.0))
    }

    /// Inverse rotation.
    pub fn transpose(&self) -> Self {
        Self(self.0.transpose())
    }

    /// Rotates a 3D vector by this DCM: `v' = R * v`.
    pub fn rotate_vec(&self, v: [f32; 3]) -> [f32; 3] {
        let m = &self.0 .0;
        [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]
    }

    /// Rotates a 3D vector by the transpose rotation: `v' = R^T * v`.
    pub fn rotate_transposed(&self, v: [f32; 3]) -> [f32; 3] {
        let m = &self.0 .0;
        [
            m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
        ]
    }
}
