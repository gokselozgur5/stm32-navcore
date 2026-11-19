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

/// Reorthonormalizes DCM via symmetric error correction and cross product.
pub fn orthonormalize_gram_schmidt(m: &Mat<3, 3>) -> Mat<3, 3> {
    let (mut x, mut y) = (m.0[0], m.0[1]);
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
    let inv_mag_x = 1.0 / libm::sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    let inv_mag_y = 1.0 / libm::sqrtf(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    let inv_mag_z = 1.0 / libm::sqrtf(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    Mat([[
        x[0] * inv_mag_x, x[1] * inv_mag_x, x[2] * inv_mag_x
    ], [
        y[0] * inv_mag_y, y[1] * inv_mag_y, y[2] * inv_mag_y
    ], [
        z[0] * inv_mag_z, z[1] * inv_mag_z, z[2] * inv_mag_z
    ]])
}

impl Dcm {
    /// Builds a rotation from Euler angles (roll, pitch, yaw) in NED-to-Body frame.
    pub fn from_euler_ned_to_body(roll: f32, pitch: f32, yaw: f32) -> Self {
        let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
        let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));
        let (sy, cy) = (libm::sinf(yaw), libm::cosf(yaw));
        Self(Mat([
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]))
    }
}
