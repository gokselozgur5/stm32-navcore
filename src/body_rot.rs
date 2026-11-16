//! NED to Body frame rotation matrix operations.

use crate::math::Mat;
use libm::{asinf, atan2f, cosf, sinf, sqrtf};

/// Error types for NED to Body rotations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyRotError {
    /// Non-finite coordinate, vector, or matrix component.
    NonFinite,
    /// Matrix failed orthonormality verification.
    NotOrthonormal,
    /// Gimbal lock condition in Euler conversion.
    GimbalLock,
}

/// Numerical tolerance for DCM orthonormality checks.
pub const BODY_ROT_TOLERANCE: f32 = 1e-4;

/// Rotation representation between NED and Body coordinate frames.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRot {
    pub(crate) dcm_nb: Mat<3, 3>,
}

impl BodyRot {
    /// Creates an identity rotation where NED and Body frames align.
    pub fn identity() -> Self {
        Self {
            dcm_nb: Mat::identity(),
        }
    }

    /// Returns a reference to the NED-to-Body DCM (C_n_b).
    pub fn dcm_ned_to_body(&self) -> &Mat<3, 3> {
        &self.dcm_nb
    }

    /// Returns the transposed Body-to-NED DCM (C_b_n).
    pub fn dcm_body_to_ned(&self) -> Mat<3, 3> {
        self.dcm_nb.transpose()
    }
}

impl BodyRot {
    /// Computes determinant of the 3x3 rotation matrix.
    pub fn det(&self) -> f32 {
        let m = &self.dcm_nb.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Checks if inner DCM satisfies orthonormality.
    pub fn is_orthonormal(&self) -> bool {
        let p = self.dcm_nb.matmul(&self.dcm_nb.transpose());
        for i in 0..3 {
            for j in 0..3 {
                let target = if i == j { 1.0 } else { 0.0 };
                if (p.0[i][j] - target).abs() > BODY_ROT_TOLERANCE {
                    return false;
                }
            }
        }
        (self.det() - 1.0).abs() <= BODY_ROT_TOLERANCE
    }
}

impl BodyRot {
    /// Builds a rotation from an existing NED-to-Body DCM.
    pub fn from_dcm_ned_to_body(m: Mat<3, 3>) -> Result<Self, BodyRotError> {
        for row in &m.0 {
            for &val in row {
                if !val.is_finite() {
                    return Err(BodyRotError::NonFinite);
                }
            }
        }
        let rot = Self { dcm_nb: m };
        if !rot.is_orthonormal() {
            return Err(BodyRotError::NotOrthonormal);
        }
        Ok(rot)
    }

    /// Builds a rotation from an existing Body-to-NED DCM.
    pub fn from_dcm_body_to_ned(m: Mat<3, 3>) -> Result<Self, BodyRotError> {
        Self::from_dcm_ned_to_body(m.transpose())
    }
}

impl BodyRot {
    /// Builds NED-to-Body rotation from Euler angles (roll, pitch, yaw).
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Result<Self, BodyRotError> {
        if !roll.is_finite() || !pitch.is_finite() || !yaw.is_finite() {
            return Err(BodyRotError::NonFinite);
        }
        let (sr, cr) = (sinf(roll), cosf(roll));
        let (sp, cp) = (sinf(pitch), cosf(pitch));
        let (sy, cy) = (sinf(yaw), cosf(yaw));

        let m = Mat([
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]);
        Ok(Self { dcm_nb: m })
    }
}

impl BodyRot {
    /// Extracts Euler angles (roll, pitch, yaw) in radians.
    pub fn to_euler(&self) -> Result<(f32, f32, f32), BodyRotError> {
        let m = &self.dcm_nb.0;
        let sin_pitch = -m[0][2];
        if sin_pitch.abs() > 0.9999 {
            return Err(BodyRotError::GimbalLock);
        }
        let pitch = asinf(sin_pitch);
        let roll = atan2f(m[1][2], m[2][2]);
        let yaw = atan2f(m[0][1], m[0][0]);
        Ok((roll, pitch, yaw))
    }
}

impl BodyRot {
    /// Rotates vector from NED frame to Body frame: v_b = C_n_b * v_n.
    pub fn rotate_ned_to_body(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        if !v[0].is_finite() || !v[1].is_finite() || !v[2].is_finite() {
            return Err(BodyRotError::NonFinite);
        }
        let m = &self.dcm_nb.0;
        Ok([
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ])
    }
}

impl BodyRot {
    /// Rotates vector from Body frame to NED frame: v_n = C_b_n * v_b.
    pub fn rotate_body_to_ned(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        if !v[0].is_finite() || !v[1].is_finite() || !v[2].is_finite() {
            return Err(BodyRotError::NonFinite);
        }
        let m = &self.dcm_nb.0;
        Ok([
            m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
        ])
    }
}

impl BodyRot {
    /// Transforms body accelerometer specific force to NED linear acceleration.
    ///
    /// Computes `a_ned = C_b_n * f_body + [0, 0, g_z]`.
    pub fn transform_accel_to_ned(
        &self,
        specific_force_body: [f32; 3],
        gravity_ned_z: f32,
    ) -> Result<[f32; 3], BodyRotError> {
        if !gravity_ned_z.is_finite() {
            return Err(BodyRotError::NonFinite);
        }
        let mut a_ned = self.rotate_body_to_ned(specific_force_body)?;
        a_ned[2] += gravity_ned_z;
        Ok(a_ned)
    }
}

impl BodyRot {
    fn normalize_vec(v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        let mag_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        if mag_sq <= 1e-12 || !mag_sq.is_finite() {
            return Err(BodyRotError::NotOrthonormal);
        }
        let inv_mag = 1.0 / sqrtf(mag_sq);
        Ok([v[0] * inv_mag, v[1] * inv_mag, v[2] * inv_mag])
    }
}

impl BodyRot {
    /// Reorthonormalizes DCM via symmetric error correction and cross product.
    pub fn reorthonormalize(&self) -> Result<Self, BodyRotError> {
        let (mut x, mut y) = (self.dcm_nb.0[0], self.dcm_nb.0[1]);
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
        let r0 = Self::normalize_vec(x)?;
        let r1 = Self::normalize_vec(y)?;
        let r2 = Self::normalize_vec(z)?;
        Self::from_dcm_ned_to_body(Mat([r0, r1, r2]))
    }
}
