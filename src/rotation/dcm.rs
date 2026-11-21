//! Direction cosine matrix with an orthonormality check.

use crate::math::Mat;

/// Errors reported by rotation utilities.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RotationError {
    /// Non-finite coordinate, vector, or matrix component.
    NonFinite,
    /// The matrix is not a proper rotation or holds non-finite values.
    NotOrthonormal,
    /// Gimbal lock condition in Euler conversion.
    GimbalLock,
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
        for row in &self.0 .0 {
            for &val in row {
                if !val.is_finite() {
                    return false;
                }
            }
        }
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

    /// Returns a shared reference to the inner matrix.
    pub fn as_matrix(&self) -> &Mat<3, 3> {
        &self.0
    }

    /// Returns a mutable reference to the inner matrix.
    pub fn as_matrix_mut(&mut self) -> &mut Mat<3, 3> {
        &mut self.0
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

impl core::ops::Mul for Dcm {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self(self.0.matmul(&rhs.0))
    }
}

impl core::ops::Mul<[f32; 3]> for Dcm {
    type Output = [f32; 3];

    fn mul(self, rhs: [f32; 3]) -> [f32; 3] {
        self.rotate_vec(rhs)
    }
}

impl Dcm {
    /// Builds a rotation from an existing NED-to-Body DCM.
    pub fn from_dcm_ned_to_body(m: Mat<3, 3>) -> Result<Self, RotationError> {
        for row in &m.0 {
            for &val in row {
                if !val.is_finite() {
                    return Err(RotationError::NonFinite);
                }
            }
        }
        let rot = Self(m);
        if !rot.is_orthonormal() {
            return Err(RotationError::NotOrthonormal);
        }
        Ok(rot)
    }

    /// Builds a rotation from an existing Body-to-NED DCM.
    pub fn from_dcm_body_to_ned(m: Mat<3, 3>) -> Result<Self, RotationError> {
        Self::from_dcm_ned_to_body(m.transpose())
    }

    /// Returns a reference to the inner matrix as NED-to-Body DCM.
    pub fn dcm_ned_to_body(&self) -> &Mat<3, 3> {
        &self.0
    }

    /// Returns the transposed Body-to-NED DCM.
    pub fn dcm_body_to_ned(&self) -> Mat<3, 3> {
        self.0.transpose()
    }
}

impl Dcm {
    /// Builds NED-to-Body DCM from Euler angles (roll, pitch, yaw) in radians.
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Result<Self, RotationError> {
        if !roll.is_finite() || !pitch.is_finite() || !yaw.is_finite() {
            return Err(RotationError::NonFinite);
        }
        let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
        let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));
        let (sy, cy) = (libm::sinf(yaw), libm::cosf(yaw));

        let m = Mat([
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]);
        Ok(Self(m))
    }

    /// Extracts Euler angles (roll, pitch, yaw) in radians from NED-to-Body DCM.
    pub fn to_euler(&self) -> Result<(f32, f32, f32), RotationError> {
        let m = &self.0 .0;
        let sin_pitch = -m[0][2];
        if sin_pitch.abs() > 0.9999 {
            return Err(RotationError::GimbalLock);
        }
        let pitch = libm::asinf(sin_pitch);
        let roll = libm::atan2f(m[1][2], m[2][2]);
        let yaw = libm::atan2f(m[0][1], m[0][0]);
        Ok((roll, pitch, yaw))
    }
}
